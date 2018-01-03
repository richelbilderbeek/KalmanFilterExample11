#ifdef _WIN32
#undef __STRICT_ANSI__
#endif

#include "maindialog.h"

#include <vector>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/functional.hpp>

#include "fparser.hh"

#include "kalmanfilter.h"
#include "matrix.h"
#include "whitenoisesystem.h"

MainDialog::MainDialog(
  const int time,
  const boost::numeric::ublas::matrix<double>& control,
  const std::vector<std::string>& input,
  const boost::numeric::ublas::matrix<double>& measurement_noise,
  const boost::numeric::ublas::matrix<double>& observation,
  const boost::numeric::ublas::matrix<double>& p_first_guess,
  const boost::numeric::ublas::matrix<double>& process_noise,
  const boost::numeric::ublas::matrix<double>& state_transition,
  const boost::numeric::ublas::vector<double>& init_x_real,
  const boost::numeric::ublas::vector<double>& real_process_noise,
  const std::vector<std::string>& state_names,
  const boost::numeric::ublas::vector<double>& x_first_guess,
  const boost::numeric::ublas::vector<double>& x_real_measurement_noise)
  : m_data(
    CreateData(
      time,
      control,
      input,
      measurement_noise,
      observation,
      p_first_guess,
      process_noise,
      state_transition,
      init_x_real,
      real_process_noise,
      state_names,
      x_first_guess,
      x_real_measurement_noise
      )
    )
{



}

const boost::numeric::ublas::matrix<double> MainDialog::CreateData(
  const int time,
  const boost::numeric::ublas::matrix<double>& control,
  const std::vector<std::string>& input,
  const boost::numeric::ublas::matrix<double>& measurement_noise,
  const boost::numeric::ublas::matrix<double>& observation,
  const boost::numeric::ublas::matrix<double>& p_first_guess,
  const boost::numeric::ublas::matrix<double>& process_noise,
  const boost::numeric::ublas::matrix<double>& state_transition,
  const boost::numeric::ublas::vector<double>& init_x_real,
  const boost::numeric::ublas::vector<double>& real_process_noise,
  const std::vector<std::string>& state_names,
  const boost::numeric::ublas::vector<double>& x_first_guess,
  const boost::numeric::ublas::vector<double>& x_real_measurement_noise)

{
  Matrix::Test();
  assert(state_names.size() == init_x_real.size());

  using boost::numeric::ublas::matrix;
  using boost::numeric::ublas::matrix_column;
  using boost::numeric::ublas::matrix_range;
  using boost::numeric::ublas::matrix_row;
  using boost::numeric::ublas::range;
  using boost::numeric::ublas::vector;
  const int n_states = init_x_real.size();

  //The resulting matrix, has time rows and states * three (real,measured,Kalman) columns
  matrix<double> data(time,n_states * m_n_curves_per_plot);
  assert(time == static_cast<int>(data.size1()));
  assert(n_states * m_n_curves_per_plot == static_cast<int>(data.size2()));
  assert(GetHeader(state_names).size() == data.size2());

  const boost::numeric::ublas::matrix<double> inputs = ParseInput(input,time);

  WhiteNoiseSystem s(control,init_x_real,x_real_measurement_noise,real_process_noise,state_transition);

  KalmanFilter k(control,x_first_guess,p_first_guess,measurement_noise,observation,process_noise,state_transition);

  //std::cout << "x_real,x_measured,x_Kalman,v_real,v_measured,v_Kalman,input\n";
  for (int i=0;i!=time;++i)
  {
    //A constant push the gas pedal, which results in a constant acceleration
    //const vector<double> input = Matrix::CreateVector( { 0.0, acceleration } );
    //Update reality, that is, let the real system (i.e. reality) go to its next state
    const matrix_column<const matrix<double> > input_as_matrix_column(inputs,i);
    const vector<double> input(input_as_matrix_column);
    assert(n_states == static_cast<int>(input.size()));
    s.GoToNextState(input);
    //Perform a noisy measurement
    const vector<double> z_measured = s.Measure();
    //Pass this measurement to the filter
    try
    {
      k.SupplyMeasurementAndInput(z_measured,input);
    }
    catch (std::runtime_error& e)
    {
      //Happens when innovation covariance becomes degenerate
      //(that is, its determinant is zero)
      return data;
    }
    //Display what the filter predicts
    const vector<double> x_est_last = k.Predict();
    for (int j=0; j!=n_states; ++j)
    {
      assert(i < static_cast<int>(data.size1()));
      assert((j*m_n_curves_per_plot)+2 < static_cast<int>(data.size2()));
      assert(j < static_cast<int>(s.PeekAtRealState().size()));
      assert(j < static_cast<int>(z_measured.size()));
      assert(j < static_cast<int>(x_est_last.size()));
      assert(j < static_cast<int>(input.size()));
      data(i,(j*m_n_curves_per_plot)+0) = s.PeekAtRealState()(j);
      data(i,(j*m_n_curves_per_plot)+1) = z_measured(j);
      data(i,(j*m_n_curves_per_plot)+2) = x_est_last(j);
      data(i,(j*m_n_curves_per_plot)+3) = input(j);
    }
  }
  return data;
}

const boost::numeric::ublas::vector<std::string> MainDialog::GetHeader(
  const std::vector<std::string>& state_names)
{
  const int n_states = static_cast<int>(state_names.size());
  boost::numeric::ublas::vector<std::string> v(n_states * m_n_curves_per_plot);
  for (int i=0; i!=n_states; ++i)
  {
    assert((i*m_n_curves_per_plot)+3 < static_cast<int>(v.size()));
    assert(i < static_cast<int>(state_names.size()));
    v((i*m_n_curves_per_plot)+0) = state_names[i] + "_real";
    v((i*m_n_curves_per_plot)+1) = state_names[i] + "_measured";
    v((i*m_n_curves_per_plot)+2) = state_names[i] + "_Kalman";
    v((i*m_n_curves_per_plot)+3) = state_names[i] + "_input";
  }
  assert(static_cast<int>(state_names.size()) * m_n_curves_per_plot == static_cast<int>(v.size()));
  return v;
}

const boost::numeric::ublas::matrix<double> MainDialog::ParseInput(
  const std::vector<std::string>& input,
  const int n_timesteps)
{
  const int n_rows = input.size();
  const int n_cols = n_timesteps;
  boost::numeric::ublas::matrix<double> m(n_rows,n_cols);

  for (int row=0; row!=n_rows; ++row)
  {
    const std::string& s = input[row];
    FunctionParser f;
    f.Parse(s.empty() ? "0.0" : s, "t");
    for (int col=0; col!=n_cols; ++col)
    {
      const double x = static_cast<double>(col);
      const double xs[1] = { x };
      const double y = f.Eval(xs);
      m(row,col) = y;
    }
  }
  return m;
}

