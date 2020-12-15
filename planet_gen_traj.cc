#define SEXPRESSO_OPT_OUT_PIKESTYLE
// clang-format off
#include <pwd.h>

#include <cxxopts.hpp>
#include <fmt/ostream.h>

#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/PathSimplifier.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <optional>

#include "common.hh"

#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
// clang-format on

#include "cpptoml.h"

#include "collision.hh"
#include "compositenn.hh"
#include "cspace.hh"
#include "goal.hh"
#include "initial.hh"
#include "motion.hh"
#include "planner_utils.hh"
#include "predicate.hh"
#include "rrt.hh"
#include "sampler.hh"
#include "robot.hh"
#include "specification.hh"
#include "scene.hh"
#include "output.hh"
#include "debug.hh"

namespace cspace        = planner::cspace;
namespace goal          = planner::goal;
namespace initial       = planner::initial;
namespace ob            = ompl::base;
namespace sampler       = planner::sampler;
namespace scene         = input::scene;
namespace specification = input::specification;
namespace util          = planner::util;
namespace rrt           = planner::rrt;
namespace pred          = symbolic::predicate;

#define FAIL(msg)                          \
  {                                        \
    spdlog::set_pattern("*** %^%v%$ ***"); \
    log->error(msg);                       \
    return EXIT_FAILURE;                   \
  }

// Evaluate an expression that returns an optional-like value (coerces to boolean, uses *to access
// a held value if it exists), then either print an error message and quit the program or return
// the held value
#define TRY(try_expr, msg)  \
  ({                        \
    auto result = try_expr; \
    if (!result) {          \
      FAIL(msg);            \
    }                       \
    *result;                \
  })

int main(int argc, char* argv[]) {
  cxxopts::Options opts(
  "planet", "Task and Motion Planning through a continuous semantics for symbolic predicates");
  // clang-format off
  opts.add_options()
    ("p,problem", "The problem configuration file", cxxopts::value<Str>())
    ("r,reps", "Run repeatedly for testing", cxxopts::value<unsigned int>()->default_value("1"))
    ("v,verbose", "Be verbose (display debug output)")
    ("g,greedy", "Use best-first sampling for actions", cxxopts::value<bool>()->default_value("false"))
    ("h,help", "Display this help message");
  // clang-format on
  opts.parse_positional({"problem"});
  auto args = opts.parse(argc, argv);
  if (args.count("help") > 0) {
    std::cout << opts.help();
    return EXIT_SUCCESS;
  }

  spdlog::set_pattern("[%H:%M:%S] %^[%l @ %n]%$ %v");
  auto log = spdlog::stdout_color_st("planet");

  const auto v_count = args.count("verbose");
  if (v_count == 0) {
    spdlog::set_level(spdlog::level::info);
  } else if (v_count == 1) {
    spdlog::set_level(spdlog::level::debug);
  } else if (v_count > 1) {
    spdlog::set_level(spdlog::level::trace);
  }

  if (args.count("problem") == 0) {
    FAIL("Please provide a problem configuration file");
  }

  auto problem_config = cpptoml::parse_file(args["problem"].as<Str>());
  const auto* pw      = getpwuid(getuid());
  if (!pw) {
    FAIL("Could not get information for current home directory!");
  }

  auto homedir      = Str(pw->pw_dir);
  auto tilde_helper = [&](Str tilde_path) {
    if (tilde_path[0] == '~') {
      return homedir + tilde_path.substr(1);
    }

    return tilde_path;
  };

  const auto& hyperparam_path = TRY(problem_config->get_as<Str>("params"),
                                    "Could not load parameters file from problem config!");
  auto hyperparams            = cpptoml::parse_file(tilde_helper(hyperparam_path));
  const auto& domain_path     = TRY(problem_config->get_as<Str>("domain"),
                                "Could not get domain file path from problem config!");
  const auto semantics_file   = TRY(problem_config->get_as<Str>("semantics"),
                                  "Could not get semantics file path from problem config!");
  const auto output_file      = TRY(problem_config->get_as<Str>("output"),
                               "Could not get output file path from problem config!");
  const auto perf_filename    = TRY(problem_config->get_as<Str>("perf_file"),
                                 "Could not get perf file path from problem config!");
  const auto cont_state_filename    = TRY(problem_config->get_as<Str>("cont_state_file"),
                                    "Could not get solution file path from problem config!");
  const auto disc_state_filename    = TRY(problem_config->get_as<Str>("disc_state_file"),
                                    "Could not get solution file path from problem config!");
  const auto time_filename    = TRY(problem_config->get_as<Str>("time_file"),
                                 "Could not get time file path from problem config!");
  const auto cfg_filename    = TRY(problem_config->get_as<Str>("cfg_file"),
                                 "Could not get configuration file path from problem config!");


  log->debug("Parsing domain from {}", domain_path);
  Str err;
  std::ifstream domain_file(tilde_helper(domain_path));
  std::stringstream domain_stream;
  domain_stream << domain_file.rdbuf();
  auto domain_sexp = sexpresso::parse(domain_stream.str(), err);
  if (!err.empty()) {
    log->error("Error parsing domain: {}", err);
    return EXIT_FAILURE;
  }

  const auto problem_path = TRY(problem_config->get_as<Str>("problem"),
                                "Could not get problem file path from problem config!");
  const auto scene_path =
  TRY(problem_config->get_as<Str>("scene"), "Could not get scene file path from problem config!");
  const auto obj_dir = TRY(problem_config->get_as<Str>("objdir"),
                           "Could not get object directory path from problem config!");

  log->debug("Parsing problem from {}", problem_path);
  std::ifstream problem_file(tilde_helper(problem_path));
  std::stringstream problem_stream;
  problem_stream << problem_file.rdbuf();
  auto problem_sexp = sexpresso::parse(problem_stream.str(), err);
  if (!err.empty()) {
    log->error("Error parsing problem: {}", err);
    return EXIT_FAILURE;
  }

  log->info("Loading problem and domain definitions...");
  auto spec = specification::load(&domain_sexp, &problem_sexp, semantics_file);

  if (!spec) {
    return EXIT_FAILURE;
  }

  auto& [domain, init_atoms, goal_atoms] = *spec;

  log->info("Loading scene with data files from {}.\n\nThis can take a long time if there's a lot "
            "of URDF to convert",
            obj_dir);

  const auto& scene_information = scene::load(tilde_helper(scene_path), tilde_helper(obj_dir));

  if (!scene_information) {
    return EXIT_FAILURE;
  }

  log->debug("Loaded scene and initial robot configuration");

  const auto& [objects, obstacles, robot, init_sg, workspace_bounds] = *scene_information;

  auto [conf_space, robot_space, objects_space, universe_space, discrete_space] =
  cspace::make_cspace(robot, init_sg.get(), domain, objects, obstacles, workspace_bounds);
  log->debug("Constructed configuration space");

  pred::objects   = &objects;
  pred::obstacles = &obstacles;

  IF_ACTION_LOG(auto graph_log = std::make_shared<debug::GraphLog>(init_atoms, &domain);)

  auto si = std::make_shared<ob::SpaceInformation>(conf_space);
  conf_space->setStateSamplerAllocator(std::bind(
  sampler::allocTampSampler, std::placeholders::_1, &domain, &robot IF_ACTION_LOG((, graph_log))));
  si->setMotionValidator(std::make_shared<planner::motion::UniverseMotionValidator>(si));
  ob::ScopedState<cspace::CompositeSpace> initial_state(si);
  initial::make_initial_state(initial_state,
                              init_atoms,
                              objects,
                              robot,
                              domain.eqclass_dimension_ids,
                              domain.discrete_dimension_ids,
                              si);
  initial_state->sg = init_sg.get();
  log->debug("Initializing universe map");
  planner::util::ActionLog action_log;
  planner::util::action_log = &action_log;
  sampler::action_log       = &action_log;

  // Set hyperparameters
  sampler::TampSampler::NUM_SOLVER_TRIES =
  hyperparams->get_as<unsigned int>("gd_tries").value_or(3);
  goal::MAX_SAMPLES = hyperparams->get_as<unsigned int>("max_samples").value_or(10);
  symbolic::heuristic::SUCCESS_SCALE =
  hyperparams->get_as<double>("success_scale").value_or(100.0);
  sampler::TampSampler::COIN_BIAS = hyperparams->get_as<double>("coin_bias").value_or(0.3);
  planner::util::GOAL_WEIGHT      = hyperparams->get_as<double>("goal_weight").value_or(4.0);

  sampler::TampSampler::NEURAL_SAMPLE = 
  hyperparams->get_as<int>("neural_sample").value_or(0);


  planner::util::UniverseMap universe_map(init_atoms,
                                          initial_state.get(),
                                          init_sg,
                                          goal_atoms,
                                          domain,
                                          objects_space.get(),
                                          si->getStateSpace().get(),
                                          conf_space->eqclass_space_idx,
                                          conf_space->discrete_space_idx,
                                          conf_space->objects_space_idx,
                                          robot.base_movable IF_ACTION_LOG((, graph_log)),
                                          args["greedy"].as<bool>());
  sampler::TampSampler::universe_map                     = &universe_map;
  cspace::CompositeSpace::universe_map                   = &universe_map;
  planner::motion::UniverseMotionValidator::universe_map = &universe_map;
  goal::CompositeGoal::universe_map                      = &universe_map;
  const auto blacklist_path = problem_config->get_as<Str>("blacklist");
  si->setStateValidityChecker(std::make_shared<planner::collisions::BulletCollisionChecker>(
  si,
  objects,
  obstacles,
  &robot,
  // NOTE: This is dumb, but cpptoml uses its own option type and I
  // don't want to include the entirety of cpptoml in collision just
  // for that one type in this one place
  blacklist_path ? std::make_optional(*blacklist_path) : std::nullopt,
  init_sg.get()));
  log->debug("Running space information setup...");
  si->setup();

  if (!si->satisfiesBounds(initial_state.get())) {
    log->critical("Initial state doesn't satisfy bounds!");
    return EXIT_FAILURE;
  }

  log->info("Initial state satisfies bounds");

  if (!si->isValid(initial_state.get())) {
    log->critical("Initial state doesn't pass validity check!");
    return EXIT_FAILURE;
  }

  log->info("Initial state is valid");
  log->info("Running sanity checks on configuration space...");
  conf_space->sanityChecks();

  auto goal_def   = std::make_shared<goal::CompositeGoal>(si, &domain, &goal_atoms, &robot);
  const auto reps = args["reps"].as<unsigned int>();
  for (unsigned int i = 0; i < reps; ++i) {
    universe_map.reset(init_atoms,
                       initial_state.get(),
                       init_sg,
                       domain,
                       goal_atoms,
                       objects_space.get(),
                       conf_space->objects_space_idx);
    action_log.clear();
    log->info("Running rep {} of {}", i + 1, reps);
    auto problem_def = std::make_shared<ob::ProblemDefinition>(si);
    problem_def->addStartState(initial_state);
    problem_def->setGoal(goal_def);
    log->debug(
    "Made space information pointer, goal sampler, initial state, and problem definition. "
    "Ready to run!");

    // Create and run planner
    auto planner          = std::make_shared<rrt::CompositeRRT>(si IF_ACTION_LOG((, graph_log)));
    planner->universe_map = &universe_map;
    planner->setProblemDefinition(problem_def);
    log->info("Dim is: {}", domain.num_eqclass_dims + domain.num_symbolic_dims);
    planner->setNearestNeighbors<planner::nn::CompositeNearestNeighbors>();
    double time_limit = 300.0;
    std::chrono::high_resolution_clock::time_point start;
    std::chrono::high_resolution_clock::time_point end;
    int iters                   = -1;
    const unsigned int MAX_TIME = problem_config->get_as<unsigned int>("timeout").value_or(320);
    do {
      log->info("Attempting to plan for {}s...", time_limit);
      start = std::chrono::high_resolution_clock::now();
      planner->solve(
      ob::timedPlannerTerminationCondition(time_limit, std::min(time_limit / 100.0, 0.1)));
      end = std::chrono::high_resolution_clock::now();

      ob::PlannerData pd(si);
      planner->getPlannerData(pd);
      log->debug("Planner is {} far away", pd.properties["approx goal distance REAL"]);
      if (v_count > 1) {
        debug::print_unimap_data(domain, log);
      }

      log->info("Sample count:\n\t{} uniform near\n\t{} gaussian near\n\t{} uniform "
                "total\n\t{} "
                "uniform regular\n\t{} heuristic",
                sampler::sample_counter.uniformNear,
                sampler::sample_counter.gaussian,
                sampler::sample_counter.uniformTotal,
                sampler::sample_counter.uniformNormal,
                sampler::sample_counter.heuristic);
      log->info("Valid states: {}",
                sampler::sample_counter.uniformTotal -
                (planner::collisions::oob_count + planner::collisions::self_coll_count +
                 planner::collisions::world_coll_count));
      log->info("Invalid sample count:\n\t{} out of bounds\n\t{} self collision\n\t{} world "
                "collision",
                planner::collisions::oob_count,
                planner::collisions::self_coll_count,
                planner::collisions::world_coll_count);
      log->info("Invalid end states: {}\nInvalid interpolation states: {}",
                planner::motion::invalid_end,
                planner::motion::invalid_interp);
      log->info("States too far from the tree: {}", planner::rrt::num_too_far);

      if (v_count > 0) {
        // Get state config/uni histogram
        Map<sampler::UniverseSig, Map<sampler::ConfigSig, unsigned int>> histogram;
        histogram.reserve(sampler::TampSampler::universe_map->graph.size());
        for (unsigned int j = 0; j < pd.numVertices(); ++j) {
          const auto& state = pd.getVertex(j).getState();
          const auto& uni   = util::discrete_of_state(
          state->as<cspace::CompositeSpace::StateType>()->as<cspace::CompositeSpace::StateType>(
          conf_space->eqclass_space_idx),
          conf_space->eqclass_subspace_count);
          const auto& cf = util::discrete_of_state(
          state->as<cspace::CompositeSpace::StateType>()->as<cspace::CompositeSpace::StateType>(
          conf_space->discrete_space_idx),
          conf_space->discrete_subspace_count);

          histogram[uni][cf]++;
        }

        for (const auto& [uni, data] : histogram) {
          log->debug(
          "{} ({}):", uni, debug::humanize_discrete_state(domain.eqclass_dimension_names, uni));
          for (const auto& [cf, count] : data) {
            log->debug("{}: {}, {}",
                       count,
                       cf,
                       debug::humanize_discrete_state(domain.discrete_dimension_names, cf));
          }

          std::cout << "==========================================\n";
        }
      }

      ++iters;
      time_limit *= 2;
    } while (!problem_def->hasSolution() && time_limit < MAX_TIME);

    log->debug("after planning, time_limit: {}, MAX_TIME: {}", time_limit, MAX_TIME);
    if (time_limit >= 2 * MAX_TIME || !problem_def->hasSolution()) {
      log->error("Failed to find a solution before reaching time limit!");
      std::ofstream perf_file(perf_filename, std::ios_base::app);
      perf_file << "failed"
                << ", "
                << "1200" << std::endl;
      continue;
    }

    auto solution_path = problem_def->getSolutionPath()->as<ompl::geometric::PathGeometric>();
    log->debug("interpolating the solution path...");
    solution_path->interpolate();  // interpolate so we have more data
    log->debug("after interpolating the solution path.");

    const auto num_states = solution_path->getStateCount();
    const auto time_taken =
    -(20.0 * (1 - std::pow(2, iters))) +
    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
    log->info("Found a solution with {} states in {}s!", num_states, time_taken);
    log->info("Outputting to {}", output_file);
    output::output_plan(output::construct_plan(solution_path, action_log, robot, si),
                        tilde_helper(output_file));
    log->info("Outputting performance info to {}", perf_filename);
    std::ofstream perf_file(perf_filename, std::ios_base::app);
    perf_file << num_states << ", " << time_taken << std::endl;


    // store the waypoints, the time taken
    std::ofstream cont_sol_file(cont_state_filename, std::ios_base::out);
    std::ofstream disc_sol_file(disc_state_filename, std::ios_base::out);

    for (unsigned i=0; i<solution_path->getStateCount(); i++)
    {
        log->debug("Printing solution path {}", i);
        const auto& state =  solution_path->getState(i)->as<ob::CompoundState>();
        const auto& robot_state = state->as<ob::CompoundState>(conf_space->robot_space_idx);
        const auto& robot_space = conf_space->getSubspace(conf_space->robot_space_idx)->as<ob::CompoundStateSpace>();
        // Robot: Base subspace (if exists)
        log->debug("Writing robot base subspace...");
        if (robot_space->hasSubspace(planner::cspace::BASE_SPACE)) {
            const auto& base_state =
            robot_state->as<planner::cspace::RobotBaseSpace::StateType>(robot_space->getSubspaceIndex(planner::cspace::BASE_SPACE));
            // print out the base state, which is RobotBaseSpace::StateType
            cont_sol_file << base_state->getX() << " " << base_state->getY() << " " << base_state->getZ();
            cont_sol_file << " " << base_state->rotation().value;
        }
        log->debug("Writing continuous joint subspace...");
        // Robot: Continuous joints subspaces
        for (unsigned subspace_idx = 0; subspace_idx < robot_space->getSubspaceCount(); subspace_idx++)
        {
            // check to make sure the name is not BASE_SPACE or other Joint_Space
            if (subspace_idx == robot_space->getSubspaceIndex(planner::cspace::BASE_SPACE) || 
                subspace_idx == robot_space->getSubspaceIndex(planner::cspace::JOINT_SPACE))
            {
                continue;
            }
            // otherwise we add it to the joint values
            const auto& joint_state = robot_state->as<ob::SO2StateSpace::StateType>(subspace_idx);
            cont_sol_file << " " << joint_state->value;
        }
        log->debug("Writing other joint subspace...");
        // Robot: Other joints subspaces
        const auto joint_space_idx = robot_space->getSubspaceIndex(planner::cspace::JOINT_SPACE);
        const auto& joints_state   = robot_state->as<planner::cspace::RobotJointSpace::StateType>(joint_space_idx);
        const auto& joints_space   = robot_space->getSubspace(joint_space_idx)->as<planner::cspace::RobotJointSpace>();
        for (unsigned int joint_state_i = 0; joint_state_i < joints_space->getDimension(); ++joint_state_i) {
            cont_sol_file << " " << joints_state->values[joint_state_i];
        }

        log->debug("Writing object subspace...");
        // Objects subspace
        const auto& objects_state = state->as<ob::CompoundState>(conf_space->objects_space_idx);
        for (unsigned int obj_i = 0; obj_i < conf_space->num_objects; ++obj_i) {
            const auto& object_state = objects_state->as<planner::cspace::ObjectSpace::StateType>(obj_i);
            // order: X, Y, Z, quaternion (x, y, z, w)
            cont_sol_file << " " << object_state->getX() << " " << object_state->getY() << " " << object_state->getZ();
            cont_sol_file << " " << object_state->rotation().x << " " << object_state->rotation().y << " " << object_state->rotation().z << " " << object_state->rotation().w;
        }

        log->debug("Writing universe subspace...");
        // Universe subspace
        const auto& universe_state = state->as<ob::CompoundState>(conf_space->eqclass_space_idx);
        for (unsigned int uni_i = 0; uni_i < conf_space->eqclass_subspace_count; ++uni_i) {
            const auto& uni_i_state = universe_state->as<planner::cspace::DiscreteSpace::StateType>(uni_i);
            disc_sol_file << " " << uni_i_state->value;  // need to strip the first space
        }

        log->debug("Writing config subspace...");
        // Config subspace
        const auto& config_state = state->as<ob::CompoundState>(conf_space->discrete_space_idx);
        for (unsigned int config_i = 0; config_i < conf_space->discrete_subspace_count; ++config_i) {
            const auto& config_i_state = config_state->as<planner::cspace::DiscreteSpace::StateType>(config_i);
            disc_sol_file << " " << config_i_state->value;        
        }
        cont_sol_file << std::endl;
        disc_sol_file << std::endl;
    }    
    std::ofstream cfg_file(cfg_filename, std::ios_base::out);
    cfg_file << "robot_space_idx: " << conf_space->robot_space_idx << std::endl;
    cfg_file << "joint_space_idx: " << conf_space->joint_space_idx << std::endl;
    cfg_file << "objects_space_idx: " << conf_space->objects_space_idx << std::endl;
    cfg_file << "num_objects: " << conf_space->num_objects << std::endl;
    cfg_file << "eqclass_space_idx: " << conf_space->eqclass_space_idx << std::endl;
    cfg_file << "eqclass_subspace_count: " << conf_space->eqclass_subspace_count << std::endl;
    cfg_file << "discrete_space_idx: " << conf_space->discrete_space_idx << std::endl;
    cfg_file << "discrete_subspace_count: " << conf_space->discrete_subspace_count << std::endl;

    std::ofstream time_file(time_filename, std::ios_base::out);
    time_file << time_taken << std::endl;


#ifdef LOG_ACTIONS
    graph_log->write(fmt::format("{}_{}_graph.json", tilde_helper(output_file), i));
#endif

    log->info("All done!");
    dynamic_cast<sampler::TampSampler*>(planner->sampler_.get())->cleanup();

    // if found one solution, break
    break;
  }

  return EXIT_SUCCESS;
}
