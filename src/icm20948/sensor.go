//go:build linux

// Package icm20948 implements a movementsensor for an ICM-20948, MPU-9250, or similar chip.
package icm20948

import (
	"context"
	"strconv"
	"sync"
	"time"

	"github.com/pkg/errors"
	"go.uber.org/multierr"
	commonpb "go.viam.com/api/common/v1"
	pb "go.viam.com/api/component/board/v1"
	goutils "go.viam.com/utils"
	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/gpio/gpioreg"
	"periph.io/x/conn/v3/physic"
	"periph.io/x/host/v3"

	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/components/board/genericlinux/buses"
	"go.viam.com/rdk/grpc"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

var Model = resource.NewModel("viam-labs", "movementsensor", "icm20948")

func init() {
	resource.RegisterComponent(
		movementsensor.API,
		Model,
		resource.Registration[movementsensor.MovementSensor, *Config]{Constructor: newSensor})
}

type icm20948 struct {
	resource.Named
	mu      sync.RWMutex
	analogs map[string]*wrappedAnalog
	pwms    map[string]pwmSetting
	logger  logging.Logger

	cancelCtx               context.Context
	cancelFunc              func()
	activeBackgroundWorkers sync.WaitGroup
}

func newSensor(
	ctx context.Context,
	_ resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (board.Board, error) {
	if _, err := host.Init(); err != nil {
		logger.Warnf("error initializing periph host", "error", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())
	b := sysfsBoard{
		Named:      conf.ResourceName().AsNamed(),
		logger:     logger,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,

		analogs: map[string]*wrappedAnalog{},
		// this is not yet modified during reconfiguration but maybe should be
		pwms: map[string]pwmSetting{},
	}

	if err := b.Reconfigure(ctx, nil, conf); err != nil {
		return nil, err
	}
	return &b, nil
}

func (b *sysfsBoard) Reconfigure(
	ctx context.Context,
	_ resource.Dependencies,
	conf resource.Config,
) error {
	b.mu.Lock()
	defer b.mu.Unlock()

	if err := b.reconfigureAnalogs(ctx, newConf); err != nil {
		return err
	}
	return nil
}

func (b *sysfsBoard) Close(ctx context.Context) error {
	b.mu.Lock()
	b.cancelFunc()
	b.mu.Unlock()
	b.activeBackgroundWorkers.Wait()
	var err error
	for _, analog := range b.analogs {
		err = multierr.Combine(err, analog.Close(ctx))
	}
	return err
}
