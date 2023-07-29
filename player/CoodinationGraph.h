#ifndef KEEPAWAY_PLAYER_COORDINATIONGRAPH_H
#define KEEPAWAY_PLAYER_COORDINATIONGRAPH_H

#include <string>
#include <stack>
#include <vector>
#include "BasicPlayer.h"
#include "LinearSarsaLearner.h"
#include <limits.h>
#include <eigen3/Eigen/Dense>

namespace cg
{
    class deepcg;
}

namespace std
{

    const string &to_string(cg
                            : deepcg *m);

}

namespace cg
{
    class Memory
    {
    private:
        Memory();

    public:
        static Memory &ins();

        void resetState();

        std::string to_string();

        bool bAlive;
        int agentIdx;
        double state[MAX_RL_STATE_VARS]; // current state -- indexed by K0..Kn
        int ballControlState[11];        // current ball state -- indexed by K0..Kn
        ObjectT teammates[11];           // current mapping from index to teammates
        ObjectT opponents[11];           // current mapping from index to opponents

    private:
        std::vector<string> stack; // self call stack

    public:
        const vector<string> &getStack() const;

        void PushStack(const string &s);

        void PopStack();

        size_t ballControlHash();
    };

    class deepcg
    {
    private:
        int num_nodes;
        int n_feat;
        int n_hid;
        int n_out;
        Eigen::MatrixXd adj;
        std::vector<Eigen::MatrixXd> weight_vec;
    }

}
