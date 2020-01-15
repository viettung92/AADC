#ifndef __REGULATOR_H__
#define __REGULATOR_H__
class Regulator
{
	private:
		long int lasttime;
		bool first_run;
		double prop,integ,diff;
		double sum, result;
		double damped_val;
		double damping;
	public:
		void reset(double p, double i, double d, double damp)
		{
			prop=p;
			integ=i;
			diff=d;
			damping=damp;
			
			first_run = true;
			sum=0;
			result=0.0;
		}

		void reset()
		{
			reset(prop,integ,diff,damping);
		}

		Regulator(double p, double i, double d, double damp)
		{
			reset(p,i,d,damp);
		}

		void put(double val, long int time_msec)
		{
			double deriv;
			double last_damped_val;


			if (first_run)
			{
				damped_val = val;
				last_damped_val = damped_val;
				lasttime = time_msec - 1; // prevent divison by zero
				first_run = false;
			}
			else
			{
				last_damped_val = damped_val;
				damped_val = damping * damped_val + (1-damping) * val;
			}

			double delta_t = (time_msec - lasttime) / 1000.;
			lasttime = time_msec;

			sum += val * delta_t;
			deriv = (damped_val - last_damped_val) / delta_t;

			result = prop * val   +   integ * sum   +   diff * deriv;
		}

		double get()
		{
			return result;
		}
};
#endif
