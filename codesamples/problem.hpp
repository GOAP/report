#pragma once

#include <algorithm>
#include <iostream>
#include <vector>
#include <map>
#include <assert.h>

#include "goap/printable.hpp"
#include "goap/fact.hpp"
#include "goap/action.hpp"

namespace Planner {
    class Problem: public Printable {
        State state;
        std::map<std::string, Action> actions;
        Goal goal;

        bool satisfies(State state, Goal g, bool weak=true, GroundState* ground=NULL) {
            for(auto it = g.begin(); it != g.end(); it++) {
                // Check if this fact is in the state
                bool found = false;
                for(auto its = state.begin(); its != state.end(); its++) {
                    if(its->name == it->name) {
                        found = true;
                        assert(its->params.size() == it->params.size());

                        // If any of the arguments are different (and not floating), it doesn't satisfy the goal
                        for(int i = 0; i < its->params.size(); i++) {
                            if(weak) {
                                if((its->params[i] != it->params[i]) && it->floating == false && its->floating == false)
                                    return false;
                            } else {
                                if(ground == NULL) return false;

                                // Check with the ground state
                                std::vector<std::string> arguments = matchGrounds(it->params, *ground);
                                if(arguments.size() == 0) // No relevant argument found in current ground, skip
                                    continue;

                                if(its->params[i] != arguments[i]) {
                                    //std::cout << "STRONG MATCH FAILURE: " << it->params[i] << " != " << arguments[i] << std::endl;

                                    return false;
                                }
                            }
                        }
                    }
                }

                if(!found)
                    return false;
            }

            return true;
        }

        std::vector<Action*> choose_operator(Goal g) {
            // TODO: Add weighting/heuristics?
            std::vector<Action*> result;

            for(auto it = this->actions.begin(); it != this->actions.end(); it++) {
                // Check if any of the postconditions in this actions are relevant to some predicate in g
                for(auto itp = it->second.postconditions.begin(); itp != it->second.postconditions.end(); itp++) {
                    for(auto itg = g.begin(); itg != g.end(); itg++) {
                        if(itp->name == itg->name)
                            result.push_back(&it->second);
                    }

                }
            }

            return result;
        }

        std::vector<Fact> update_state(State s, Plan p, GroundState ground) {
            for(auto it = p.begin(); it != p.end(); it++) {
                // First, find an action with the same name as this fact.
                auto act_it = this->actions.find((*it).name);

                // If no action with a matching name is found, ignore this step.
                if(act_it == this->actions.end())
                    continue;

                // Otherwise, apply it to the state.
                Action act = act_it->second;
                s = act.engage(s, ground);
            }

            return s;
        }

        std::vector<GroundState> generateGroundStates(Action* act, State s, Goal g) {
            std::vector<GroundState> result;
            std::vector<std::string> values;

            // Extract all values
            for(auto it = s.begin(); it != s.end(); it++) {
                for(auto itp = it->params.begin(); itp != it->params.end(); itp++) {
                    values.push_back(*itp);
                }
            }
            for(auto it = g.begin(); it != g.end(); it++) {
                for(auto itp = it->params.begin(); itp != it->params.end(); itp++) {
                    values.push_back(*itp);
                }
            }

            std::sort(values.begin(), values.end()); // don't ask

            do { 
                int ctr = 0;
                GroundState target;

                // Find all combinations
                for(auto itp = act->params.begin(); itp != act->params.end(); itp++) {
                    //std::cout << *itp << " " << values[ctr] << std::endl;
                    target[*itp] = values[ctr++];
                }

                result.push_back(target);
            } while(std::next_permutation(values.begin(), values.end()));

            return result;
        }

        Plan* solve_(State s, Goal g, int depth = 0, GroundState* stepGround = NULL) {
            std::string padding;
            Plan* p = new Plan;

            for(int i = 0; i < depth; i++) {
                padding += "  ";
            }

            std::cout << padding << "Subproblem state: " << std::endl;
            for(auto it = s.begin(); it != s.end(); it++) {
                std::cout << padding << " - ";
                it->print();
                std::cout << std::endl;
            }

            std::cout << padding << "Subgoal: " << std::endl;
            for(auto it = g.begin(); it != g.end(); it++) {
                std::cout << padding <<" - ";
                it->print();
                std::cout << std::endl;
            }

            if(stepGround != NULL) {
                std::cout << padding << "Ground: " << std::endl;
                for(auto it = stepGround->begin(); it != stepGround->end(); it++) {
                    std::cout << padding << " - " << it->first << ": " << it->second;
                    std::cout << std::endl;
                }
            }

            // Vector of applicable operators. 
            auto candidate_operators = choose_operator(g);

            while(true) {
                bool weakMatch = stepGround == NULL;

                if(satisfies(s, g, weakMatch, stepGround)) {
                    std::cout << padding << ">> SOLUTION << state: " << std::endl;
                    for(auto it = s.begin(); it != s.end(); it++) {
                        std::cout << padding << " - ";
                        it->print();
                        std::cout << std::endl;
                    }

                    // Mission accomplished! \o/
                    return p;
                }

                if(candidate_operators.size() == 0) {
                    return NULL;
                }

                // Get the first candidate operator, and pop it from the list
                auto candidate = candidate_operators[0];
                candidate_operators.erase(candidate_operators.begin());

                // Generate possible ground states
                std::vector<GroundState> grounds = this->generateGroundStates(candidate, s, g);
                auto ground = grounds.end();
                Plan* subplan = NULL;
                bool found = false;

                for(auto it = grounds.begin(); it != grounds.end(); it++) {
                    ground = it; 
                    GroundState* copiedGround = &(*it);

                    // Get a plan that solves candidate's preconditions with the current ground candidate
                    subplan = this->solve_(s, candidate->preconditions, depth + 1, copiedGround);
                    if(subplan == NULL) {
                        //std::cout << "Subplan invalid, continuing to next ground." << std::endl;
                        continue;
                    }

                    // Check if this ground state satisifes the subgoal.
                    // Run all actions in the subplan and update h
                    State h = update_state(s, *subplan, *ground);
                    // Run the current action candidate and modify the state
                    h = candidate->engage(h, *ground);

                    if(satisfies(h, g, weakMatch, copiedGround)) {
                        found = true;
                        s = h;
                        break;
                    } else {
                        std::cout << "Nope, invalid final state." << std::endl;
                    }
                }

                if(found == false) // None of the ground states are valid: no solution
                    return NULL;

                // Add all steps in subplan to the Master Plan
                for(auto it = subplan->begin(); it != subplan->end(); it++) {
                    p->push_back(*it);
                }

                // Add the current action to the plan, matching params to ground.
                std::vector<std::string> arguments = matchGrounds(candidate->params, *ground);
                p->push_back(Step(candidate->name, arguments)); // Housekeeping
            }
        }

    public:
        Problem(std::vector<Action> _actions, State _state, Goal _goal):
            state(_state), goal(_goal) {

            for(auto it = _actions.begin(); it != _actions.end(); it++) {
                this->actions.insert(std::pair<std::string, Action>(it->name, *it)); // Thanks C++
            }
        };

        void print() {
            std::cout << "Available actions: " << std::endl;
            for(auto it = this->actions.begin(); it != this->actions.end(); it++) {
                std::cout << " - ";
                it->second.print();
                std::cout << std::endl;
            }
            std::cout << std::endl;

            std::cout << "Problem state: " << std::endl;
            for(auto it = this->state.begin(); it != this->state.end(); it++) {
                std::cout << " - ";
                it->print();
                std::cout << std::endl;
            }
            std::cout << std::endl;

            std::cout << "Goal: " << std::endl;
            for(auto it = this->goal.begin(); it != this->goal.end(); it++) {
                std::cout << " - ";
                it->print();
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }

        Plan* solve() {
            return this->solve_(this->state, this->goal);
        }

    };
};
