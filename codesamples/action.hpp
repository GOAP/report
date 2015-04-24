#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <assert.h>

#include "goap/printable.hpp"
#include "goap/fact.hpp"

namespace Planner {
    class Action: public Printable {
    public:
        std::string name;
        std::vector<std::string> params;
        std::vector<Fact> preconditions;
        std::vector<Fact> postconditions;

        Action(std::string _name, std::vector<std::string> _params, std::vector<Fact> _preconditions, std::vector<Fact> _postconditions):
            name(_name), params(_params), preconditions(_preconditions), postconditions(_postconditions) {};

        void print() {
            std::cout << this->name << "(";
            for(auto it = this->params.begin(); it != this->params.end(); it++) {
                std::cout << *it << ", ";
            }
            std::cout << ")";

            std::cout << ", preconditions: ";
            for(auto it = this->preconditions.begin(); it != this->preconditions.end(); it++) {
                it->print();
            }

            std::cout << ", postconditions: ";
            for(auto it = this->postconditions.begin(); it != this->postconditions.end(); it++) {
                it->print();
            }
        }

        State engage(State s, GroundState ground) {
            // Remove negated postconditions from state, add new facts to the state
            for(auto it = this->postconditions.begin(); it != this->postconditions.end(); it++) {
                std::string name = it->name;
                if(name[0] == '!') { // Remove this from the state
                    name.erase(0, 1);

                    auto position = s.end();
                    for(auto stateIt = s.begin(); stateIt != s.end(); stateIt++)
                        if(stateIt->name == name)
                            position = stateIt;

                    if(position != s.end())
                        s.erase(position);

                } else { // Addit to the state
                    if(it->floating == false) { // Trivial case. Not floating? Add it directly.
                        s.push_back(*it);
                    } else { // Otherwise, match it with the ground.
                        // At this point, we know the name of the fact we want to create,
                        // but we need to match up the arguments with the supplied ground state.
                        std::vector<std::string> arguments = matchGrounds(it->params, ground);

                        // Got all of this postcondition's params matched up to their ground state.
                        // All that's left is to instantiate a matching Fact and add it to the state.
                        s.push_back(Fact(name, arguments));
                    }
                }
            }

            return s;
        }
    };
};
