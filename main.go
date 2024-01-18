package main

import (
	"context"

	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/module"
	"go.viam.com/utils"

	"github.com/viam-labs/icm20948/src/icm20948"
)

func mainWithArgs(ctx context.Context, args []string, logger logging.Logger) (err error) {
	modalModule, err := module.NewModuleFromArgs(ctx, logger)
	if err != nil {
		return err
	}
	modalModule.AddModelFromRegistry(ctx, movementsensor.API, icm20948.Model)

	err = modalModule.Start(ctx)
	if err != nil {
		return err
	}
	defer modalModule.Close(ctx)

	<-ctx.Done()
	return nil
}

func main() {
	utils.ContextualMain(mainWithArgs, logging.NewLogger("icm20948"))
}
