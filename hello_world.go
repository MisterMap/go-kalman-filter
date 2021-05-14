package main

import (
	"fmt"
	"gonum.org/v1/gonum/mat"
	"math/rand"
)

func main() {
	data := make([]float64, 36)
	for i := range data {
		data[i] = rand.NormFloat64()
	}
	//a := mat.NewDense(6, 6, data)
	//b := mat.NewDense(6, 6, data)
	//rand.NormFloat64()
	//mat.NewVecDense()
	emp1 := employee{}
	fmt.Printf("Emp1: %+v\n", emp1)

	emp2 := employee{name: "Sam", age: 31, salary: 2000}
	fmt.Printf("Emp2: %+v\n", emp2)

	emp3 := employee{
		name:   "Sam",
		age:    31,
		salary: 2000,
	}
	fmt.Printf("Emp3: %+v\n", emp3)

	emp4 := employee{
		name: "Sam",
		age:  31,
	}
	fmt.Printf("Emp4: %+v\n", emp4)
}

type RandomMotionParameters struct {
	motionNoise      float64
	measurementNoise float64
	initialState     mat.VecDense
}

func generateData(pointCount int, parameters RandomMotionParameters,
	timeDelta float64) ([]mat.VecDense, []mat.VecDense) {
	state := parameters.initialState
	var measurements []mat.VecDense
	var states []mat.VecDense
	var acceleration float64
	for i := 0; i < pointCount; i++ {
		acceleration = rand.NormFloat64() * parameters.motionNoise
		state := mat.NewVecDense(2, []float64{state.AtVec(0) + state.AtVec(1)*timeDelta + acceleration*timeDelta*timeDelta/2,
			state.AtVec(1) + acceleration*timeDelta})
		measurement := mat.NewVecDense(1, []float64{state.AtVec(0) + rand.NormFloat64()*parameters.measurementNoise})
		measurements = append(measurements, *measurement)
		states = append(states, *state)
	}
	return measurements, states
}

type employee struct {
	name   string
	age    int
	salary int
}
