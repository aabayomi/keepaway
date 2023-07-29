
//
// Created by Abayomi Adekanmbi  on 07/18/2023
// University of Texas at Austin
//
// This file contains the implementation of the Graph Convolutional Network (GCN) model

#pragma once

#ifndef GCN_MODEL
#define GCN_MODEL

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "Layers.h"
#include <iostream>
#include <vector>
#include "HierarchicalFSM.h"
#include "ChoicePoint.h"

using namespace std;
class gcn
{
public:
    Conv conv1;
    Conv conv2;
    gcn(Eigen::MatrixXd A, int nfeat, int nhid, int nout, std::vector<Eigen::MatrixXd> weights) : conv1(nfeat, nhid, A, weights[0], true), conv2(nhid, nout, A, weights[1], true)
    {
    }

    Eigen::MatrixXd forward(Eigen::MatrixXd x)
    {
        Eigen::MatrixXd h1 = conv1.forward(x);
        h1 = relu(h1);
        Eigen::MatrixXd h2 = conv2.forward(h1);
        h2 = relu(h2);
        return h2;
    }
};

#endif // GCN_MODEL