package kalman

import (
	"gonum.org/v1/gonum/mat"
)

type Filter struct {
	motionModel *mat.Dense
	motionErrorModel *mat.Dense
	measurementModel *mat.Dense
	measurementErrorModel *mat.Dense
}

type State struct {
	state *mat.VecDense
	covariance *mat.Dense
}

func (filter Filter) filterState(measurement mat.VecDense, previousState State) State {
	// Prediction
	var state mat.VecDense
	state.MulVec(filter.motionModel, previousState.state)
	var covariance mat.Dense
	covariance.Mul(filter.motionModel, previousState.covariance)
	covariance.Mul(&covariance, filter.motionModel.T())
	covariance.Add(&covariance, filter.motionErrorModel)


	// Filtration
	var errorCovariance mat.Dense
	errorCovariance.Mul(filter.measurementModel, &covariance)
	errorCovariance.Mul(&errorCovariance, (&covariance).T())
	errorCovariance.Add(&errorCovariance, filter.measurementErrorModel)

	var inverseErrorCovariance mat.Dense
	err := inverseErrorCovariance.Inverse(&errorCovariance)
	if err != nil {
		return State{}
	}
	var kalmanGain mat.Dense
	kalmanGain.Mul(&covariance, filter.measurementModel.T())
	kalmanGain.Mul(&kalmanGain, &inverseErrorCovariance)

	var measurementState mat.VecDense
	measurementState.MulVec(filter.measurementModel, &state)
	measurementState.SubVec(&measurement, &measurementState)
	measurementState.MulVec(&kalmanGain, &measurementState)
	state.AddVec(&state, &measurementState)

	var measurementCovariance mat.Dense
	eyeMatrix := mat.NewDiagDense(state.Len(), make([]float64, state.Len()))
	measurementCovariance.Mul(&kalmanGain, filter.measurementModel)
	measurementCovariance.Sub(eyeMatrix, &measurementCovariance)
	covariance.Mul(&measurementCovariance, &covariance)

	return State{&state, &covariance}
}

func (filter Filter) filterStates(measurements []mat.VecDense, initialState State) []State {
	currentState := initialState
	var filteredStates []State
	for _, measurement := range measurements {
		currentState = filter.filterState(measurement, currentState)
		filteredStates = append(filteredStates, currentState)
	}
	return filteredStates
}
