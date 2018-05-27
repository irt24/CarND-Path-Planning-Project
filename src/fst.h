#ifndef FST_H
#define FST_H

#include <set>

using namespace std;  // Bad practice :(

const int kLaneSizeMeters = 4;
const int kSafeDistanceMeters = 30;
const double kInf = std::numeric_limits<double>::max();

struct Predictions {
  double ego_s;
  vector<double> other_cars_s;
  vector<double> other_cars_d;
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

namespace {
double GetMiddleOfLane(Lane lane) {
  return kLaneSizeMeters / 2 + kLaneSizeMeters * lane;
}

bool CarIsOnLane(Lane lane, double d) {
  double middle = GetMiddleOfLane(lane);
  return (d > middle - kLaneSizeMeters / 2) && (d < middle + kLaneSizeMeters / 2);
}
}  // end anonymous namespace

class FST {
 public:
  FST(Lane initial_lane) : current_lane(initial_lane) {}

  // The transition function.
  void NextState(const Predictions& predictions) {
    State min_cost_state;
    double min_cost = kInf; 

    // Find the state with minimum cost.
    for (const State& state : GetPossibleNextStates()) {
      double cost = 0.9 * CostOfCollision(predictions, state) + 0.2 * CostOfChange(state);
      if (cost < min_cost) {
        min_cost = cost;
        min_cost_state = state;
      }
    };

    current_state = min_cost_state; 
    current_lane = GetLaneForState(current_state);
  }

  Lane current_lane;
  State current_state = LANE_KEEP;

 private:
  set<State> GetPossibleNextStates() const {
    set<State> valid_states = valid_transitions.find(current_state)->second;
    if (current_lane == Lane::LEFT) {
      valid_states.erase(State::LANE_CHANGE_LEFT);
    } else if (current_lane == Lane::RIGHT) {
      valid_states.erase(State::LANE_CHANGE_RIGHT);
    }
    return valid_states;
  }

  Lane GetLaneForState(const State& state) const {
    switch (state) {
      case State::LANE_KEEP:
      case State::SLOW_DOWN: return current_lane;
      case State::LANE_CHANGE_LEFT:
        return static_cast<Lane>(static_cast<int>(current_lane) - 1);
      case State::LANE_CHANGE_RIGHT:
        return static_cast<Lane>(static_cast<int>(current_lane) + 1);
    }
  }

  double CostOfCollision(const Predictions& predictions,
                         const State& next_state) const {
    Lane next_lane = GetLaneForState(next_state);

    for (int i = 0; i < predictions.other_cars_d.size(); i++) {
      if (CarIsOnLane(next_lane, predictions.other_cars_d[i])) {
        double s = predictions.other_cars_s[i];
        double ego_s = predictions.ego_s;
        if ((s > ego_s) && (s - ego_s < kSafeDistanceMeters)) {
          return next_state == State::SLOW_DOWN ? 0.0 : 1.0;
        }
      }
    }

    return 0.0;
  }

  double CostOfChange(const State& next_state) const {
    switch (next_state) {
      case State::LANE_KEEP: return 0.0;
      case State::SLOW_DOWN: return 1.0;
      case State::LANE_CHANGE_RIGHT:
      case State::LANE_CHANGE_LEFT: return 0.5;
    }
  }

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
