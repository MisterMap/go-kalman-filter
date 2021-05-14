package kalman

import (
	"gonum.org/v1/gonum/mat"
	"math"
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
	fields1 := fields{mat.NewDense(2, 2, []float64{1, 0, 0, 1}),
		mat.NewDense(2, 2, []float64{1, 0, 0, 1}),
		mat.NewDense(1, 2, []float64{1, 0}),
		mat.NewDense(1, 1, []float64{1})}
	args1 := args{*mat.NewVecDense(1, []float64{1}),
		State{mat.NewVecDense(2, []float64{1, 1}),
			mat.NewDense(2, 2, []float64{1, 0, 0, 1})}}
	want1 := State{mat.NewVecDense(2, []float64{1, 1}),
		mat.NewDense(2, 2, []float64{1, 0, 0, 1})}
	tests := []struct {
		name   string
		fields fields
		args   args
		want   State
	}{{"simpleFilter",
		fields1,
		args1,
		want1},
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

func TestAbs(t *testing.T) {
	got := math.Abs(-1)
	if got != 1 {
		t.Errorf("Abs(-1) = %v; want 1", got)
	}
}

func TestMakeSimpleKalmanFilter(t *testing.T) {

}
