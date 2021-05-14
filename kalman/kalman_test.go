package kalman

import (
	"gonum.org/v1/gonum/mat"
	"reflect"
	"testing"
)

func TestFilter_FilterState(t *testing.T) {
	type fields struct {
		MotionModel           *mat.Dense
		MotionErrorModel      *mat.Dense
		MeasurementModel      *mat.Dense
		MeasurementErrorModel *mat.Dense
	}
	type args struct {
		measurement   mat.VecDense
		previousState State
	}
	tests := []struct {
		name   string
		fields fields
		args   args
		want   State
	}{
		// TODO: Add test cases.
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			filter := Filter{
				MotionModel:           tt.fields.MotionModel,
				MotionErrorModel:      tt.fields.MotionErrorModel,
				MeasurementModel:      tt.fields.MeasurementModel,
				MeasurementErrorModel: tt.fields.MeasurementErrorModel,
			}
			if got := filter.FilterState(tt.args.measurement, tt.args.previousState); !reflect.DeepEqual(got, tt.want) {
				t.Errorf("FilterState() = %v, want %v", got, tt.want)
			}
		})
	}
}

func TestFilter_FilterStates(t *testing.T) {
	type fields struct {
		MotionModel           *mat.Dense
		MotionErrorModel      *mat.Dense
		MeasurementModel      *mat.Dense
		MeasurementErrorModel *mat.Dense
	}
	type args struct {
		measurements []mat.VecDense
		initialState State
	}
	tests := []struct {
		name   string
		fields fields
		args   args
		want   []State
	}{
		// TODO: Add test cases.
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			filter := Filter{
				MotionModel:           tt.fields.MotionModel,
				MotionErrorModel:      tt.fields.MotionErrorModel,
				MeasurementModel:      tt.fields.MeasurementModel,
				MeasurementErrorModel: tt.fields.MeasurementErrorModel,
			}
			if got := filter.FilterStates(tt.args.measurements, tt.args.initialState); !reflect.DeepEqual(got, tt.want) {
				t.Errorf("FilterStates() = %v, want %v", got, tt.want)
			}
		})
	}
}
