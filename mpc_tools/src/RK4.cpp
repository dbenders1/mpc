
// #include <mpc_tools/RK4.h>

namespace Helpers {

namespace RK4 {

template <class StateType>
void Observer<StateType>::operator()(const StateType& x_cur,
                                     const double t_cur) noexcept {
  x.push_back(x_cur);
  time.push_back(t_cur);
}

template <class StateType>
void Observer<StateType>::clear() {
  x.clear();
  time.clear();
}

// RK4 integrator class
template <class StateType, class System>
void Integrator<StateType, System>::integrate_const(System& sys,
                                                    StateType& x_cur,
                                                    const double t_cur,
                                                    const double t_final,
                                                    const double dt) {
  // Clear any results left in observer
  observer_.clear();

  // Integrate
  boost::numeric::odeint::integrate_const(rk4_stepper_, std::ref(sys), x_cur,
                                          t_cur, t_final, dt,
                                          std::ref(observer_));

  // Result can be accessed via observer_.x
}

// Predifine what templates are going to be used if you already know beforehand
// This to give the compiler already information of what we need
template class System<std::vector<double>>;
template class Observer<std::vector<double>>;
template class Integrator<std::vector<double>, System<std::vector<double>>>;

// template void Observer<std::vector<double>>::operator()(const
// std::vector<double> &, const double); template void
// Integrator<std::vector<double>>::integrate_const(System<std::vector<double>>&,
// std::vector<double>, const double, const double, const double);

}; /* namespace RK4 */
}; /* namespace Helpers */
