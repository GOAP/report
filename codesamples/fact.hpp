#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <assert.h>

#include "goap/printable.hpp"

namespace Planner {
    class Fact: public Printable {
    public:
        std::string name;
        std::vector<std::string> params;
        bool floating;

        Fact(std::string _name, std::vector<std::string> _params, bool _floating=false):
            name(_name), params(_params), floating(_floating) {};

        void reify() {
            this->floating = false;
        }

        void print() {
            std::cout << this->name << "(";

            for(auto it = this->params.begin(); it != this->params.end(); it++) {
                std::cout << *it << ", ";
            }

            std::cout << ") (floating: " << (this->floating == 1 ? "true" : "false") << ")";
        }
    };

    typedef Fact Step;
    typedef std::vector<Fact> Goal;
    typedef std::vector<Step> Plan;
    typedef std::vector<Fact> State;
    typedef std::map<std::string, std::string> GroundState;

    // Match a list of parameters to their ground state.
    std::vector<std::string> matchGrounds(std::vector<std::string> params, GroundState ground) {
        std::vector<std::string> arguments;

        for(auto param = params.begin(); param != params.end(); param++) {
            // Find this parameter in the ground state.
            auto value = ground.find(*param);
            if(value != ground.end()) {
                arguments.push_back(value->second);
            }
        }

        return arguments;
    }

}
