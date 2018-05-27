#ifndef FST_H
#define FST_H

#include <set>

using namespace std;  // Bad practice :(

const int kSafeDistanceMeters = 30;
const double kInf = std::numeric_limits<double>::max();

struct Predictions {
  double ego_s;
  std::vector<double> other_car_s;
};

class Trajectory {
};

enum Lane {
  LEFT = 0,
  MIDDLE = 1,
  RIGHT = 2, 
};

enum State {
  LANE_KEEP,
  SLOW_DOWN,
  LANE_CHANGE_LEFT,
  LANE_CHANGE_RIGHT,
};

class FST {
 public:
  FST(Lane initial_lane) : current_lane(initial_lane) {}

  double cost1(const Predictions& predictions, const State& next_state) const {
    if (current_lane == Lane::LEFT) {
      // Keep the lane.
      return next_state == State::LANE_KEEP ? 0.0 : kInf; 
    }

    if (next_state == State::LANE_CHANGE_LEFT) {
      // Figure out whether any of the other cars are too close.
      for (const double other_s : predictions.other_car_s) {
        if ((other_s > predictions.ego_s) &&
            (other_s - predictions.ego_s < kSafeDistanceMeters)) {
          cout << "Too close. Changing lanes to left." << endl;
          return 0.0;
        }
      }
    }

    return kInf;
  }

  static double cost2() { return 0.0; }
  static double cost3() { return 0.0; }

  // The transition function.
  void NextState(const Predictions& predictions) {
    State min_cost_state;
    double min_cost = kInf; 

    // Find the state with minimum cost.
    for (const State& state : GetPossibleNextStates()) {
      double cost = cost1(predictions, state); 
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_state = state;
      }
    };

    TransitionTo(min_cost_state);
  }

  Trajectory GenerateTrajectory(State state) {
    // TODO: Implement this method.
    return Trajectory();
  }

  set<State> GetPossibleNextStates() const {
    set<State> valid_states = valid_transitions.find(current_state)->second;
    if (current_lane == Lane::LEFT) {
      valid_states.erase(State::LANE_CHANGE_LEFT);
    } else if (current_lane == Lane::RIGHT) {
      valid_states.erase(State::LANE_CHANGE_RIGHT);
    }
    return valid_states;
  }

  void TransitionTo(const State& state) {
    current_state = state;
    if (state == State::LANE_CHANGE_LEFT) {
      current_lane = static_cast<Lane>(static_cast<int>(current_lane) - 1);
    } else if (state == State::LANE_CHANGE_RIGHT) {
      current_lane = static_cast<Lane>(static_cast<int>(current_lane) + 1);
    }
  }

  Lane current_lane;
  State current_state = LANE_KEEP;

  const set<State> all_states =
      {LANE_KEEP,
       SLOW_DOWN,
       LANE_CHANGE_LEFT,
       LANE_CHANGE_RIGHT};
  const map<State, set<State>> valid_transitions =
      {{LANE_KEEP, all_states},
       {SLOW_DOWN, all_states},
       {LANE_CHANGE_LEFT, {LANE_KEEP}},
       {LANE_CHANGE_RIGHT, {LANE_KEEP}}};
};

#endif  // FST_H
