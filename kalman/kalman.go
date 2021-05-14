package kalman

import (
	"gonum.org/v1/gonum/mat"
)

type Filter struct {
	MotionModel           *mat.Dense
	MotionErrorModel      *mat.Dense
	MeasurementModel      *mat.Dense
	MeasurementErrorModel *mat.Dense
}

type State struct {
	State      *mat.VecDense
	Covariance *mat.Dense
}

func (filter Filter) FilterState(measurement mat.VecDense, previousState State) State {
	// Prediction
	var state mat.VecDense
	state.MulVec(filter.MotionModel, previousState.State)
	var covariance mat.Dense
	covariance.Mul(filter.MotionModel, previousState.Covariance)
	covariance.Mul(&covariance, filter.MotionModel.T())
	covariance.Add(&covariance, filter.MotionErrorModel)

	// Filtration
	var errorCovariance mat.Dense
	errorCovariance.Mul(filter.MeasurementModel, &covariance)
	errorCovariance.Mul(&errorCovariance, (&covariance).T())
	errorCovariance.Add(&errorCovariance, filter.MeasurementErrorModel)

	var inverseErrorCovariance mat.Dense
	err := inverseErrorCovariance.Inverse(&errorCovariance)
	if err != nil {
		return State{}
	}
	var kalmanGain mat.Dense
	kalmanGain.Mul(&covariance, filter.MeasurementModel.T())
	kalmanGain.Mul(&kalmanGain, &inverseErrorCovariance)

	var measurementState mat.VecDense
	measurementState.MulVec(filter.MeasurementModel, &state)
	measurementState.SubVec(&measurement, &measurementState)
	measurementState.MulVec(&kalmanGain, &measurementState)
	state.AddVec(&state, &measurementState)

	var measurementCovariance mat.Dense
	eyeMatrix := mat.NewDiagDense(state.Len(), make([]float64, state.Len()))
	measurementCovariance.Mul(&kalmanGain, filter.MeasurementModel)
	measurementCovariance.Sub(eyeMatrix, &measurementCovariance)
	covariance.Mul(&measurementCovariance, &covariance)

	return State{&state, &covariance}
}

func (filter Filter) FilterStates(measurements []mat.VecDense, initialState State) []State {
	currentState := initialState
	var filteredStates []State
	for _, measurement := range measurements {
		currentState = filter.FilterState(measurement, currentState)
		filteredStates = append(filteredStates, currentState)
	}
	return filteredStates
}

func MakeSimpleKalmanFilter(timeDelta float64, motionNoise float64, measurementNoise float64) Filter {
	motionModel := mat.NewDense(2, 2, []float64{1, 0, timeDelta, 1})

	stateMotionErrorModel := mat.NewDense(2, 1, []float64{timeDelta * timeDelta / 2, timeDelta})
	var motionErrorModel mat.Dense
	motionErrorModel.Mul(stateMotionErrorModel, stateMotionErrorModel.T())
	motionErrorModel.Scale(motionNoise*motionNoise, &motionErrorModel)

	measurementModel := mat.NewDense(1, 2, []float64{1, 0})

	measurementErrorModel := mat.NewDense(1, 1, []float64{measurementNoise * measurementNoise})
	filter := Filter{
		MotionModel:           motionModel,
		MotionErrorModel:      &motionErrorModel,
		MeasurementModel:      measurementModel,
		MeasurementErrorModel: measurementErrorModel,
	}
	return filter
}
