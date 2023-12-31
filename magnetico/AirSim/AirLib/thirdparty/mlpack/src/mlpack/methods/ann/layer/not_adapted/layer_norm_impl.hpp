/**
 * @file methods/ann/layer/layer_norm_impl.hpp
 * @author Shikhar Jaiswal
 *
 * Implementation of the Layer Normalization class.
 *
 * mlpack is free software; you may redistribute it and/or modify it under the
 * terms of the 3-clause BSD license.  You should have received a copy of the
 * 3-clause BSD license along with mlpack.  If not, see
 * http://www.opensource.org/licenses/BSD-3-Clause for more information.
 */

#ifndef MLPACK_METHODS_ANN_LAYER_LAYERNORM_IMPL_HPP
#define MLPACK_METHODS_ANN_LAYER_LAYERNORM_IMPL_HPP

// In case it is not included.
#include "layer_norm.hpp"

namespace mlpack {


template<typename InputType, typename OutputType>
LayerNormType<InputType, OutputType>::LayerNormType() :
    size(0),
    eps(1e-8),
    loading(false)
{
  // Nothing to do here.
}

template <typename InputType, typename OutputType>
LayerNormType<InputType, OutputType>::LayerNormType(
    const size_t size, const double eps) :
    size(size),
    eps(eps),
    loading(false)
{
  weights.set_size(size + size, 1);
}

template<typename InputType, typename OutputType>
void LayerNormType<InputType, OutputType>::SetWeights(
    typename OutputType::elem_type* weightsPtr)
{
  gamma = OutputType(weightsPtr, size, 1, false, false);
  beta = OutputType(weightsPtr + gamma.n_elem, size, 1, false, false);

  if (!loading)
  {
    gamma.fill(1.0);
    beta.fill(0.0);
  }

  loading = false;
}

// Copy Constructor.
template <typename InputType, typename OutputType>
LayerNormType<InputType, OutputType>::LayerNormType(
													const LayerNormType& layer) :
size(layer.size),
eps(layer.eps),
loading(false),
gamma(layer.gamma),
beta(layer.beta),
weights(layer.weights),
mean(layer.mean),
variance(layer.variance),
normalized(layer.normalized),
inputMean(layer.inputMean)
{
	// Implement the copy constructor to create a deep copy of the layer.
	// You may need to copy all relevant member variables and allocate memory
	// for new objects.
}

// Move Constructor.
template <typename InputType, typename OutputType>
LayerNormType<InputType, OutputType>::LayerNormType(
													LayerNormType&& layer) :
size(std::move(layer.size)),
eps(std::move(layer.eps)),
loading(false),
gamma(std::move(layer.gamma)),
beta(std::move(layer.beta)),
weights(std::move(layer.weights)),
mean(std::move(layer.mean)),
variance(std::move(layer.variance)),
normalized(std::move(layer.normalized)),
inputMean(std::move(layer.inputMean))
{
	// Implement the move constructor to transfer ownership of resources.
	// You may need to transfer member variables and reset the source object.
}

// Copy assignment operator.
template <typename InputType, typename OutputType>
LayerNormType<InputType, OutputType>& LayerNormType<InputType, OutputType>::operator=(
																					  const LayerNormType& layer)
{
	if (this != &layer)
	{
		// Implement the copy assignment operator to copy the layer.
		// You may need to release any resources held by the current object,
		// copy member variables, and allocate new resources if needed.
		size = layer.size;
		eps = layer.eps;
		loading = false;
		gamma = layer.gamma;
		beta = layer.beta;
		weights = layer.weights;
		mean = layer.mean;
		variance = layer.variance;
		normalized = layer.normalized;
		inputMean = layer.inputMean;
	}
	return *this;
}

// Move assignment operator.
template <typename InputType, typename OutputType>
LayerNormType<InputType, OutputType>& LayerNormType<InputType, OutputType>::operator=(
																					  LayerNormType&& layer)
{
	if (this != &layer)
	{
		// Implement the move assignment operator to transfer ownership of resources.
		// You may need to transfer member variables and reset the source object.
		size = std::move(layer.size);
		eps = std::move(layer.eps);
		loading = false;
		gamma = std::move(layer.gamma);
		beta = std::move(layer.beta);
		weights = std::move(layer.weights);
		mean = std::move(layer.mean);
		variance = std::move(layer.variance);
		normalized = std::move(layer.normalized);
		inputMean = std::move(layer.inputMean);
	}
	return *this;
}


template<typename InputType, typename OutputType>
void LayerNormType<InputType, OutputType>::Forward(
    const InputType& input, OutputType& output)
{
  mean = arma::mean(input, 0);
  variance = arma::var(input, 1, 0);

  // Normalize the input.
  output = input.each_row() - mean;
  inputMean = output;
  output.each_row() /= arma::sqrt(variance + eps);

  // Reused in the backward and gradient step.
  normalized = output;

  // Scale and shift the output.
  output.each_col() %= gamma;
  output.each_col() += beta;
}

template<typename InputType, typename OutputType>
void LayerNormType<InputType, OutputType>::Backward(
    const InputType& input, const OutputType& gy, OutputType& g)
{
  const OutputType stdInv = 1.0 / arma::sqrt(variance + eps);

  // dl / dxhat.
  const OutputType norm = gy.each_col() % gamma;

  // sum dl / dxhat * (x - mu) * -0.5 * stdInv^3.
  const OutputType var = arma::sum(norm % inputMean, 0) %
      arma::pow(stdInv, 3.0) * -0.5;

  // dl / dxhat * 1 / stdInv + variance * 2 * (x - mu) / m +
  // dl / dmu * 1 / m.
  g = (norm.each_row() % stdInv) + (inputMean.each_row() %
      var * 2 / input.n_rows);

  // sum (dl / dxhat * -1 / stdInv) + variance *
  // (sum -2 * (x - mu)) / m.
  g.each_row() += arma::sum(norm.each_row() % -stdInv, 0) / input.n_rows;
}

template<typename InputType, typename OutputType>
void LayerNormType<InputType, OutputType>::Gradient(
    const InputType& /* input */,
    const OutputType& error,
    OutputType& gradient)
{
  gradient.set_size(size + size, 1);

  // Step 5: dl / dy * xhat.
  gradient.submat(0, 0, gamma.n_elem - 1, 0) = arma::sum(normalized % error, 1);

  // Step 6: dl / dy.
  gradient.submat(gamma.n_elem, 0, gradient.n_elem - 1, 0) =
      arma::sum(error, 1);
}

template<typename InputType, typename OutputType>
template<typename Archive>
void LayerNormType<InputType, OutputType>::serialize(
    Archive& ar, const uint32_t /* version */)
{
  ar(cereal::base_class<Layer<InputType>>(this));

  ar(CEREAL_NVP(size));
  ar(CEREAL_NVP(eps));

  // Ensure that we don't set the values of the weights if we have already
  // learned them.
  if (cereal::is_loading<Archive>())
  {
    loading = true;
  }
}

} // namespace mlpack

#endif
