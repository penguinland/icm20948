//go:build linux

// Package icm20948 implements a movementsensor for an ICM-20948, MPU-9250, or similar chip.
package icm20948

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	ahrs "github.com/tracktum/go-ahrs"
	"go.uber.org/multierr"

	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"

	"gonum.org/v1/gonum/num/quat"

	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

var Model = resource.NewModel("viam-labs", "demo", "mpu9250")

type Config struct {
}

// Validate ensures all parts of the config are valid.
func (cfg *Config) Validate(path string) ([]string, error) {
	return []string{}, nil
}

func init() {
	resource.RegisterComponent(movementsensor.API, Model, resource.Registration[movementsensor.MovementSensor, *Config]{
		Constructor: NewIMU,
	})
}

type mpu struct {
	resource.Named
	angularVelocity spatialmath.AngularVelocity
	orientation     spatialmath.Orientation
	acceleration    r3.Vector
	magnetometer    r3.Vector
	mpuHandle, magHandle  i2c.Dev
	logger          logging.Logger
	aScale, gScale      float64
	gXCal, gYCal, gZCal float64
	ahrs            ahrs.Madgwick

	readMu          sync.RWMutex
	mu              sync.Mutex
	cancelFunc              func()
	activeBackgroundWorkers sync.WaitGroup
}

func (i *mpu) AngularVelocity(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
	i.readMu.RLock()
	defer i.readMu.RUnlock()
	return i.angularVelocity, nil
}

func (i *mpu) Orientation(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
	i.readMu.RLock()
	defer i.readMu.RUnlock()
	return i.orientation, nil
}

func (i *mpu) LinearAcceleration(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	i.readMu.RLock()
	defer i.readMu.RUnlock()
	return i.acceleration, nil
}

func (i *mpu) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	i.readMu.RLock()
	defer i.readMu.RUnlock()
	readings := make(map[string]interface{})
	readings["linear_acceleration"] = i.acceleration
	readings["angular_velocity"] = i.angularVelocity
	readings["orientation"] = i.orientation
	return readings, nil
}

// NewIMU creates a new mpu9250 IMU
func NewIMU(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger logging.Logger) (movementsensor.MovementSensor, error) {
	if _, err := host.Init(); err != nil {
		return nil, err
	}

	bus, err := i2creg.Open("1")
	if err != nil {
		return nil, err
	}

	mpuHandle := i2c.Dev{bus, MPU6050_ADDR}
	magHandle := i2c.Dev{bus, AK8963_ADDR}


	// TODO Make these config variables
	dev := &mpu{mpuHandle: mpuHandle, magHandle: magHandle, logger: logger,
		Named: conf.ResourceName().AsNamed(),
		aScale: 2.0,
		gScale: 250.0,
		gXCal: -2.6,
		gYCal: -1.6,
		gZCal: -0.3,
	}

	dev.ahrs = ahrs.NewMadgwick(0.1, 100) // 0.1 beta, 100hz update

	errs := multierr.Combine(dev.startMPU(ctx), dev.startMag(ctx))
	if errs != nil {
		return nil, errs
	}

	var cancelCtx context.Context
	cancelCtx, dev.cancelFunc = context.WithCancel(ctx)
	waitCh := make(chan struct{})
	// startTime := time.Now()
	// ticks := 0
	dev.activeBackgroundWorkers.Add(1)
	utils.PanicCapturingGo(func() {
		defer dev.activeBackgroundWorkers.Done()
		timer := time.NewTicker(time.Duration(float64(time.Second)/100)) // 100hz
		defer timer.Stop()
		close(waitCh)
		for {
			select {
			case <-cancelCtx.Done():
				return
			default:
			}
			select {
			case <-cancelCtx.Done():
				return
			case <-timer.C:
				// ticks++
				// if ticks % 100 == 0 {
				// 	logger.Debugf("MPU Time: %+v", time.Now().Sub(startTime) / time.Duration(ticks))
				// }
				err := dev.doRead(ctx)
				if err != nil {
					logger.Error(err)
				}
			}
		}
	})
	<-waitCh

	return dev, nil
}

func (i *mpu) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	// newConf, err := resource.NativeConfig[*Config](conf)
	// if err != nil {
	// 	return err
	// }

	return nil
}

func (i *mpu) Properties(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
	return &movementsensor.Properties{
		AngularVelocitySupported:    true,
		LinearAccelerationSupported: true,
		OrientationSupported: true,
	}, nil
}

func (i *mpu) LinearVelocity(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	return r3.Vector{}, movementsensor.ErrMethodUnimplementedLinearVelocity
}

func (i *mpu) CompassHeading(ctx context.Context, extra map[string]interface{}) (float64, error) {
	return 0, movementsensor.ErrMethodUnimplementedCompassHeading
}

func (i *mpu) Position(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
	return geo.NewPoint(0, 0), 0, movementsensor.ErrMethodUnimplementedPosition
}

func (i *mpu) Accuracy(ctx context.Context, extra map[string]interface{}) (map[string]float32, error) {
	return map[string]float32{}, movementsensor.ErrMethodUnimplementedAccuracy
}




func (i *mpu) startMPU(ctx context.Context) error {
	// i.mu.Lock()
	// defer i.mu.Unlock()

	// Reset
	var errs error
	errs = multierr.Combine(errs, writeByteData(i.mpuHandle, PWR_MGMT_1, 0b10000000))
	if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	// Set clocksource to optimal
	errs = multierr.Combine(errs, writeByteData(i.mpuHandle, PWR_MGMT_1, 0b00000001))
	if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	// Enable i2c passthrough for compass
	errs = multierr.Combine(errs, writeByteData(i.mpuHandle, INT_PIN_CFG, 0b00000010))
	if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }



	// errs = multierr.Combine(errs, h.WriteByteData(ctx, SMPLRT_DIV, 0))
	// if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	// errs = multierr.Combine(errs, h.WriteByteData(ctx, CONFIG, 0))
	// if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	// errs = multierr.Combine(errs, h.WriteByteData(ctx, GYRO_CONFIG, 0))
	// if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	// errs = multierr.Combine(errs, h.WriteByteData(ctx, USER_CTRL, 0))
	// if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	// errs = multierr.Combine(errs, h.WriteByteData(ctx, INT_ENABLE, 1))
	// if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	return errs
}

func (i *mpu) startMag(ctx context.Context) error {
	// i.mu.Lock()
	// defer i.mu.Unlock()

	var errs error
	errs = multierr.Combine(errs, writeByteData(i.magHandle, AK8963_CNTL, 0))
	time.Sleep(100 * time.Millisecond)

	res  := byte(0b0001)  // 0b0001 = 16-bit
	rate := byte(0b0110)  // 0b0010 = 8 Hz, 0b0110 = 100 Hz

	mode := (res <<4)+rate // bit conversion

	errs = multierr.Combine(errs, writeByteData(i.magHandle, AK8963_CNTL, mode))
	if !utils.SelectContextOrWait(ctx, 100 * time.Millisecond) { return errs }

	return errs
}

func (i *mpu) Close(ctx context.Context) error {
	i.cancelFunc()
	i.activeBackgroundWorkers.Wait()
	// i.mpuHandle.Close()
	// i.magHandle.Close()
	return nil
}

func (i *mpu) doRead(ctx context.Context) error {
	// i.mu.Lock()
	// defer i.mu.Unlock()

	var errs error
	x, err := i.readRawBits(ctx, ACCEL_XOUT_H)
	errs = multierr.Combine(errs, err)
	y, err := i.readRawBits(ctx, ACCEL_YOUT_H)
	errs = multierr.Combine(errs, err)
	z, err := i.readRawBits(ctx, ACCEL_ZOUT_H)
	errs = multierr.Combine(errs, err)
	acceleration := r3.Vector{X: i.scaleAcceleration(x), Y: i.scaleAcceleration(y), Z: i.scaleAcceleration(z)}



	x, err = i.readRawBits(ctx, GYRO_XOUT_H)
	errs = multierr.Combine(errs, err)
	y, err = i.readRawBits(ctx, GYRO_YOUT_H)
	errs = multierr.Combine(errs, err)
	z, err = i.readRawBits(ctx, GYRO_ZOUT_H)
	errs = multierr.Combine(errs, err)
	angularVelocity := spatialmath.AngularVelocity{X: i.scaleGyro(x) +  i.gXCal, Y: i.scaleGyro(y) + i.gYCal, Z: i.scaleGyro(z) + i.gZCal}

	if errs != nil {
		return errs
	}

	for loop := 0; loop < 15; loop++  {
		var errs error
		x, err := i.readRawBitsMag(ctx, HXH)
		errs = multierr.Combine(errs, err)
		y, err := i.readRawBitsMag(ctx, HYH)
		errs = multierr.Combine(errs, err)
		z, err := i.readRawBitsMag(ctx, HZH)
		errs = multierr.Combine(errs, err)

		status, err := readByteData(i.magHandle, AK8963_ST2)
		errs = multierr.Combine(errs, err)
		if status != 0b10000 {
			if loop > 10 {
				return multierr.Combine(errs, fmt.Errorf("imu magnetometer overflow: %x", status))
			}
			continue
		}

		if errs == nil {
			i.magnetometer = r3.Vector{X: i.scaleMag(x), Y: i.scaleMag(y), Z: i.scaleMag(z)}
			break
		}
	}

	q := i.ahrs.Update9D(
		angularVelocity.X * (math.Pi/180), // rad/s to deg/s
		angularVelocity.Y * (math.Pi/180),
		angularVelocity.Z * (math.Pi/180),
		acceleration.X / 1000, // mm/s^2 to m/s^2
		acceleration.Y / 1000,
		acceleration.Z / 1000,
		i.magnetometer.X / 100,  // microteslas to gauss
		i.magnetometer.Y / 100,
		i.magnetometer.Z / 100,
	)

	// i.logger.Debugf("SMURF %+v", q)

	i.readMu.Lock()
	myQuat := spatialmath.Quaternion(quat.Number{q[0], q[1], q[2], q[3]})
	i.orientation = &myQuat
	i.acceleration = acceleration
	i.angularVelocity = angularVelocity
	i.readMu.Unlock()
	return nil
}


func (i *mpu) scaleAcceleration(raw int16) float64 {
	// Default 16-bit range is +/- 2G
	// Scaling factor: g (in m/s^2) * scale range / math.MaxInt16
	return float64(raw) * ((9806.5 * i.aScale) / math.MaxInt16)
}

func (i *mpu) scaleGyro(raw int16) float64 {
	// Default 16-bit range is +/- 250 deg/s
	return float64(raw) * ((i.gScale) / math.MaxInt16)
}

func (i *mpu) scaleMag(raw int16) float64 {
	// Fixed scale, 0.15 uT/bit (4900 +/- / math.MaxInt16)
	return float64(raw) * 0.15
}

func (i *mpu) readRawBits(ctx context.Context, register uint8) (int16, error) {
	// xH, err1 := i.mpuHandle.ReadByteData(ctx, register)
	// xL, err2 := i.mpuHandle.ReadByteData(ctx, register+1)

	x, err := readWordData(i.mpuHandle, register)
	return int16((uint16(x[0])<<8)|uint16(x[1])), err
}

func (i *mpu) readRawBitsMag(ctx context.Context, register uint8) (int16, error) {
	// xH, err1 := i.magHandle.ReadByteData(ctx, register)
	// xL, err2 := i.magHandle.ReadByteData(ctx, register-1)

	x, err := readWordData(i.magHandle, register-1)
	return int16((uint16(x[1])<<8)|uint16(x[0])), err
}


func swapbytes16(d uint16) uint16 {
	return (d << 8) | (d >> 8)
}

func writeByteData(dev i2c.Dev, reg, data byte) error {
	_, err := dev.Write([]byte{reg, data})
	return err
}

func readByteData(dev i2c.Dev, reg byte) (byte, error) {
	val := make([]byte, 1)
	err := dev.Tx([]byte{reg}, val)
	return val[0], err
}

func readWordData(dev i2c.Dev, reg byte) ([]byte, error) {
	val := make([]byte, 2)
	err := dev.Tx([]byte{reg}, val)
	return val, err
}
