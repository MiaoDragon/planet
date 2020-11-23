#include "rrtstar.hh"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/PathGeometric.h>

// clang-format off
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

namespace planner::rrtstar {
namespace {
  auto log = spdlog::stdout_color_st("RRTstar");
}

util::UniverseMap* CompositeRRTstar::universe_map = nullptr;
unsigned long num_too_far                     = 0;

ob::PlannerStatus CompositeRRTstar::solve(const ob::PlannerTerminationCondition& ptc) {
  checkValidity();
  bool symCost = opt_->isSymmetric();
  ob::Goal* goal = pdef_->getGoal().get();
  auto* goal_s   = dynamic_cast<ob::GoalSampleableRegion*>(goal);

  while (const ob::State* st = pis_.nextStart()) {
	auto* motion = new Motion(si_);
	si_->copyState(motion->state, st);
	nn_->add(motion);
  }

  if (nn_->size() == 0) {
	OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
	return ob::PlannerStatus::INVALID_START;
  }

  if (!sampler_) sampler_ = si_->allocStateSampler();

  OMPL_INFORM("%s: Starting planning with %u states already in datastructure",
			  getName().c_str(),
			  nn_->size());

	OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

	if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ || useNewStateRejection_) &&
		!si_->getStateSpace()->isMetricSpace())
		OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
				  "the triangle inequality. "
				  "You may need to disable pruning or rejection.",
				  getName().c_str(), si_->getStateSpace()->getName().c_str());

  const ob::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

  //Motion* solution  = nullptr;
  //Motion* approxsol = nullptr;
  //double approxdif  = std::numeric_limits<double>::infinity();
  Motion *approxGoalMotion = nullptr;
  double approxDist = std::numeric_limits<double>::infinity();
  
  auto* rmotion     = new Motion(si_);
  ob::State* rstate = rmotion->state;
  ob::State* xstate = si_->allocState();



  std::vector<Motion *> nbh;

  std::vector<ob::Cost> costs;
  std::vector<ob::Cost> incCosts;
  std::vector<std::size_t> sortedCostIndices;

  std::vector<int> valid;
  unsigned int rewireTest = 0;
  unsigned int statesGenerated = 0;

  if (bestGoalMotion_)
      OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                  bestCost_.value());

  if (useKNearest_)
      OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                  (unsigned int)std::ceil(k_rrt_ * std::log((double)(nn_->size() + 1u))));
  else
      OMPL_INFORM(
          "%s: Initial rewiring radius of %.2f", getName().c_str(),
          std::min(maxDistance_, r_rrt_ * std::pow(std::log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                    1 / (double)(si_->getStateDimension()))));

  // our functor for sorting nearest neighbors
  CostIndexCompare compareFn(costs, *opt_);

  while (!ptc) {
  iterations_++;
	// log->critical("Start of sample loop; {} states in NN", nn_->size());
	/* sample random state (with goal biasing) */
	//if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
  if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
            goal_s->canSample())
	  goal_s->sampleGoal(rstate);
	else
	  sampler_->sampleUniform(rstate);

	/* find closest state in the tree */
	// log->critical("Starting NN");
	Motion* nmotion   = nn_->nearest(rmotion);

  if (intermediateSolutionCallback && si_->equalStates(nmotion->state, rstate))
      continue;

	ob::State* dstate = rstate;
	// log->critical("Ending NN");

	/* find state to add */
	// log->critical("Starting distance");
	double d = si_->distance(nmotion->state, rstate);
	if (d > maxDistance_) {
	  // We set the action to nullptr because we won't actually reach the state where the action
	  // was used
	  rstate->as<util::HashableStateSpace::StateType>()->action = nullptr;
	  // log->warn("{} is greater than {}. Interpolating!", d, maxDistance_);
	  si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
	  dstate = xstate;
	  ++num_too_far;
	}

	auto* action_data = dstate->as<util::HashableStateSpace::StateType>()->action;

	// log->critical("Checking motion");
	if (si_->checkMotion(nmotion->state, dstate)) {
    // update heuristics for the current action data
	  if (action_data != nullptr) {
		action_data->update(std::get<2>(action_data->data)->update_success());
    // action has successfully been applied
		IF_ACTION_LOG(graph_log->update_success(action_data);)
	  }

	  // if (addIntermediateStates_) {
		// std::vector<ob::State*> states;
		// const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

		// if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
		//   si_->freeState(states[0]);

		// for (std::size_t i = 1; i < states.size(); ++i) {
		//   universe_map->added_state(states[i]->as<util::HashableStateSpace::StateType>());
		//   Motion* motion = new Motion;
		//   motion->state  = states[i];
		//   motion->parent = nmotion;
		//   nn_->add(motion);
		//   nmotion = motion;
		// }
	  // }

		universe_map->added_state(dstate->as<util::HashableStateSpace::StateType>());
		Motion* motion = new Motion(si_);
		si_->copyState(motion->state, dstate);
		motion->parent = nmotion;
    motion->incCost = opt_->motionCost(nmotion->state, motion->state);
    motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

    // Find nearby neighbors of the new motion
    getNeighbors(motion, nbh);


    rewireTest += nbh.size();
    ++statesGenerated;

    // cache for distance computations
    //
    // Our cost caches only increase in size, so they're only
    // resized if they can't fit the current neighborhood
    if (costs.size() < nbh.size())
    {
        costs.resize(nbh.size());
        incCosts.resize(nbh.size());
        sortedCostIndices.resize(nbh.size());
    }

    // cache for motion validity (only useful in a symmetric space)
    //
    // Our validity caches only increase in size, so they're
    // only resized if they can't fit the current neighborhood
    if (valid.size() < nbh.size())
        valid.resize(nbh.size());
    std::fill(valid.begin(), valid.begin() + nbh.size(), 0);


    // RRTstar
    if (delayCC_)
    {
        // calculate all costs and distances
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
            incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
            costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
        }

        // sort the nodes
        //
        // we're using index-value pairs so that we can get at
        // original, unsorted indices
        for (std::size_t i = 0; i < nbh.size(); ++i)
            sortedCostIndices[i] = i;
        std::sort(sortedCostIndices.begin(), sortedCostIndices.begin() + nbh.size(), compareFn);

        // collision check until a valid motion is found
        //
        // ASYMMETRIC CASE: it's possible that none of these
        // neighbors are valid. This is fine, because motion
        // already has a connection to the tree through
        // nmotion (with populated cost fields!).
        for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
              i != sortedCostIndices.begin() + nbh.size(); ++i)
        {
            if (nbh[*i] == nmotion ||
                ((!useKNearest_ || si_->distance(nbh[*i]->state, motion->state) < maxDistance_) &&
                  si_->checkMotion(nbh[*i]->state, motion->state)))
            {
                motion->incCost = incCosts[*i];
                motion->cost = costs[*i];
                motion->parent = nbh[*i];
                valid[*i] = 1;
                break;
            }
            else
                valid[*i] = -1;
        }
    }
    else  // if not delayCC
    {
        motion->incCost = opt_->motionCost(nmotion->state, motion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
        // find which one we connect the new state to
        for (std::size_t i = 0; i < nbh.size(); ++i)
        {
            if (nbh[i] != nmotion)
            {
                incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                if (opt_->isCostBetterThan(costs[i], motion->cost))
                {
                    if ((!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                        si_->checkMotion(nbh[i]->state, motion->state))
                    {
                        motion->incCost = incCosts[i];
                        motion->cost = costs[i];
                        motion->parent = nbh[i];
                        valid[i] = 1;
                    }
                    else
                        valid[i] = -1;
                }
            }
            else
            {
                incCosts[i] = motion->incCost;
                costs[i] = motion->cost;
                valid[i] = 1;
            }
        }
    }

    if (useNewStateRejection_)
    {
        if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
        {
            nn_->add(motion);
            motion->parent->children.push_back(motion);
        }
        else  // If the new motion does not improve the best cost it is ignored.
        {
            si_->freeState(motion->state);
            delete motion;
            continue;
        }
    }
    else
    {
        // add motion to the tree
        nn_->add(motion);
        motion->parent->children.push_back(motion);
    }


    // check solution and rewire
    bool checkForSolution = false;
    for (std::size_t i = 0; i < nbh.size(); ++i)
    {
        if (nbh[i] != motion->parent)
        {
            ob::Cost nbhIncCost;
            if (symCost)
                nbhIncCost = incCosts[i];
            else
                nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
            ob::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
            if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
            {
                bool motionValid;
                if (valid[i] == 0)
                {
                    motionValid =
                        (!useKNearest_ || si_->distance(nbh[i]->state, motion->state) < maxDistance_) &&
                        si_->checkMotion(motion->state, nbh[i]->state);
                }
                else
                {
                    motionValid = (valid[i] == 1);
                }

                if (motionValid)
                {
                    // //due to rewire, we need to NULL the near node's action since we don't know which one to apply
                    // // TODO: update the transition action to be for the new state/node
                    // nbh[i]->state->as<util::HashableStateSpace::StateType>()->action = nullptr;

                    // Remove this node from its parent list
                    removeFromParent(nbh[i]);

                    // Add this node to the new parent
                    nbh[i]->parent = motion;
                    nbh[i]->incCost = nbhIncCost;
                    nbh[i]->cost = nbhNewCost;
                    nbh[i]->parent->children.push_back(nbh[i]);

                    // Update the costs of the node's children
                    updateChildCosts(nbh[i]);

                    checkForSolution = true;
                }
            }
        }
    }

    // Add the new motion to the goalMotion_ list, if it satisfies the goal
    double distanceFromGoal;
    if (goal->isSatisfied(motion->state, &distanceFromGoal))
    {
        motion->inGoal = true;
        goalMotions_.push_back(motion);
        checkForSolution = true;
    }

    // Checking for solution or iterative improvement
    if (checkForSolution)
    {
        bool updatedSolution = false;
        if (!bestGoalMotion_ && !goalMotions_.empty())
        {
            // We have found our first solution, store it as the best. We only add one
            // vertex at a time, so there can only be one goal vertex at this moment.
            bestGoalMotion_ = goalMotions_.front();
            bestCost_ = bestGoalMotion_->cost;
            updatedSolution = true;

            OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                        "vertices in the graph)",
                        getName().c_str(), bestCost_.value(), iterations_, nn_->size());
        }
        else
        {
            // We already have a solution, iterate through the list of goal vertices
            // and see if there's any improvement.
            for (auto &goalMotion : goalMotions_)
            {
                // Is this goal motion better than the (current) best?
                if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                {
                    bestGoalMotion_ = goalMotion;
                    bestCost_ = bestGoalMotion_->cost;
                    updatedSolution = true;

                    // Check if it satisfies the optimization objective, if it does, break the for loop
                    if (opt_->isSatisfied(bestCost_))
                    {
                        break;
                    }
                }
            }
        }

        if (updatedSolution)
        {
            if (useTreePruning_)
            {
                pruneTree(bestCost_);
            }

            if (intermediateSolutionCallback)
            {
                std::vector<const ob::State *> spath;
                Motion *intermediate_solution =
                    bestGoalMotion_->parent;  // Do not include goal state to simplify code.

                // Push back until we find the start, but not the start itself
                while (intermediate_solution->parent != nullptr)
                {
                    spath.push_back(intermediate_solution->state);
                    intermediate_solution = intermediate_solution->parent;
                }

                intermediateSolutionCallback(this, spath, bestCost_);
            }
        }
    }

    // Checking for approximate solution (closest state found to the goal)
    if (goalMotions_.size() == 0 && distanceFromGoal < approxDist)
    {
        approxGoalMotion = motion;
        approxDist = distanceFromGoal;
    }
  }
    // terminate if a sufficient solution is found
    if (bestGoalMotion_ && opt_->isSatisfied(bestCost_))
        break;
}

  // Add our solution (if it exists)
  Motion *newSolution = nullptr;
  if (bestGoalMotion_)
  {
      // We have an exact solution
      newSolution = bestGoalMotion_;
  }
  else if (approxGoalMotion)
  {
      // We don't have a solution, but we do have an approximate solution
      newSolution = approxGoalMotion;
  }
  // No else, we have nothing

  // Add what we found
  if (newSolution)
  {
      ptc.terminate();
      // construct the solution path
      std::vector<Motion *> mpath;
      Motion *iterMotion = newSolution;
      while (iterMotion != nullptr)
      {
          mpath.push_back(iterMotion);
          iterMotion = iterMotion->parent;
      }

      // set the solution path
      auto path(std::make_shared<og::PathGeometric>(si_));
      for (int i = mpath.size() - 1; i >= 0; --i)
          path->append(mpath[i]->state);

      // Add the solution path.
      ob::PlannerSolution psol(path);
      psol.setPlannerName(getName());

      // If we don't have a goal motion, the solution is approximate
      if (!bestGoalMotion_)
          psol.setApproximate(approxDist);

      // Does the solution satisfy the optimization objective?
      psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
      pdef_->addSolutionPath(psol);
  }
  // No else, we have nothing

  si_->freeState(xstate);
  if (rmotion->state)
      si_->freeState(rmotion->state);
  delete rmotion;

  OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
              "%.3f",
              getName().c_str(), statesGenerated, rewireTest, goalMotions_.size(), bestCost_.value());

  // We've added a solution if newSolution == true, and it is an approximate solution if bestGoalMotion_ == false
  return {newSolution != nullptr, bestGoalMotion_ == nullptr};
}

}  // namespace planner::rrtstar
