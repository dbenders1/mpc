#ifndef MPC_TOOLS_RK4_H
#define MPC_TOOLS_RK4_H

#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include <boost/numeric/odeint/stepper/runge_kutta4.hpp>
#include <functional>
#include <vector>

namespace Helpers {
namespace RK4 {

// System class
template <class StateType>
class System {
 public:
  System() {};
  ~System() {};

  virtual void operator()(const StateType &x, StateType &dxdt, double t) = 0;
};

// Observer class
template <class StateType>
class Observer {
 public:
  Observer() {};
  ~Observer() {};

  void operator()(const StateType &x_cur, const double t_cur) noexcept {
    x.push_back(x_cur);
    time.push_back(t_cur);
  }

  void clear() {
    x.clear();
    time.clear();
  }

  std::vector<StateType> x;
  std::vector<double> time;
};

// RK4 integrator class
template <class StateType, class System>
class Integrator {
 public:
  Integrator() {};
  ~Integrator() {};

  void integrate_const(System &sys, StateType x_cur, const double t_cur,
                       const double t_final, const double dt) {
    // Clear any results left in observer
    observer_.clear();

    // Integrate
    boost::numeric::odeint::integrate_const(rk4_stepper_, std::ref(sys), x_cur,
                                            t_cur, t_final, dt,
                                            std::ref(observer_));

    // Result can be accessed via observer_.x
  }

  Observer<StateType> observer_;

 private:
  boost::numeric::odeint::runge_kutta4<StateType> rk4_stepper_;
};

}; /* namespace RK4 */
}; /* namespace Helpers */

#endif