
#ifndef SIMPLE_NN_ACTIVATION_HPP
#define SIMPLE_NN_ACTIVATION_HPP
#include <eigen3/Eigen/Core>

namespace simple_nn {

    struct Linear {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            return input;
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            return Eigen::MatrixXd::Ones(input.rows(), input.cols());
        }
    };

    struct Sigmoid {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            return 1. / (1. + (-input).array().exp());
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd value = f(input);
            return (value.array() * (1. - value.array()));
        }
    };

    struct Softmax {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd tmp = input.array().exp();
            Eigen::VectorXd t = tmp.colwise().sum();
            tmp.array().rowwise() /= t.transpose().array();
            return tmp;
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd value = f(input);
            return (value.array() * (1. - value.array()));
        }
    };

    struct Gaussian {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            return (-input.array().square()).exp();
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd value = f(input);
            return -2. * input.array() * value.array();
        }
    };

    struct Swish {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            return input.array() * Sigmoid::f(input).array();
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd value = f(input);
            return value.array() + Sigmoid::f(input).array() * (1. - value.array());
        }
    };

    struct Tanh {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            return input.array().tanh();
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd value = f(input);
            return 1. - value.array().square();
        }
    };

    struct Cos {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            return input.array().cos();
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            return -input.array().sin();
        }
    };

    struct Sin {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            return input.array().sin();
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            return input.array().cos();
        }
    };

    struct ReLU {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd z = Eigen::MatrixXd::Zero(input.rows(), input.cols());
            Eigen::MatrixXd output = (input.array() > 0).select(input, z);

            return output;
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd z = Eigen::MatrixXd::Zero(input.rows(), input.cols());
            Eigen::MatrixXd o = Eigen::MatrixXd::Ones(input.rows(), input.cols());
            Eigen::MatrixXd output = (input.array() > 0).select(o, z);

            return output;
        }
    };

    struct Multiply {
        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd output = input.colwise().prod();

            return output;
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            Eigen::MatrixXd output(input.rows(), input.cols());
            // TO-DO: Make this faster
            for (int r = 0; r < output.rows(); r++) {
                for (int c = 0; c < output.cols(); c++) {
                    Eigen::VectorXd col = input.col(c);
                    col[r] = 1.;
                    output(r, c) = col.prod();
                }
            }

            return output;
        }
    };

    struct Divide {
        static constexpr double epsilon = 1e-16;

        static Eigen::MatrixXd f(const Eigen::MatrixXd& input)
        {
            // We only divide two numbers
            assert(input.rows() == 2);
            Eigen::MatrixXd output = input.row(0).array() / (input.row(1).array() + epsilon);

            return output;
        }

        static Eigen::MatrixXd df(const Eigen::MatrixXd& input)
        {
            // We only divide two numbers
            assert(input.rows() == 2);
            Eigen::MatrixXd output(input.rows(), input.cols());

            output.row(0) = 1. / (input.row(1).array() + epsilon);
            output.row(1) = -input.row(0).array() / (input.row(1).array() + epsilon).square();

            return output;
        }
    };
} // namespace simple_nn

#endif