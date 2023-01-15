#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <torch/torch.h>
#include <iostream>

// N is batch size; D_in is input dimension
// H is hidden dimension; D_out is output dimension
const int64_t N = 64;
const int64_t D_in = 1000;
const int64_t H = 100;
const int64_t D_out = 4;
const double mean = 0.0;
const float dev  = 0.02;

struct TwoLayerNet : torch::nn::Module {
  TwoLayerNet() : linear1(D_in, H), linear2(H, D_out) {
    register_module("linear1", linear1);
    register_module("linear2", linear2);
  }
  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(linear1->forward(x));
    x = linear2->forward(x);
    return x;
  }


  void initialize_weights(torch::nn::Module& module) {
      torch::NoGradGuard no_grad;

      if (auto* linear = module.as<torch::nn::Linear>()) {
        linear->weight.normal_(mean, dev);
      }
    }
  torch::nn::Linear linear1;
  torch::nn::Linear linear2;
};

// void initialize_weights(torch::nn::Module& module) {
//     torch::NoGradGuard no_grad;

//     if (auto* linear = module.as<torch::nn::Linear>()) {
//       linear->weight.normal_(mean, dev);
//     }
//   }

#endif