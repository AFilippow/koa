
#include "vectormath.h"

#define ALPHA_Z 0.8		//original values are 0.75 and alpha_z / 4
#define BETA_Z ALPHA_Z/4
#define ALPHA_V 1.0
#define ALPHA_W 1.0

//#define LAMBDA 0.5
//#define BETA 2.0
#define DISTANCE_SHORTENER 0.05//7 ///We cut this value in m off of every Distance to an obstacle to account for the finite size of the end effector
#define CALCULATED_GRADIENT 1
#define PHASE_STOPPING 1
class xDMP {
	public:
		xDMP();
		~xDMP();
		
		void set_alpha_z(float x) {alpha_z=x;}
		void set_beta_z(float x)  {beta_z=x;}
		void set_alpha_v(float x) {alpha_v=x;}
		void set_alpha_w(float x) {alpha_w=x;}
		void set_s(std::vector<float> x) {s=x;}
		void set_g(std::vector<float> x) {g=x;}
		void set_T(float x) {T=x;}
		void set_dt(float x) {dt=x;}
		void set_tau(float x) {tau=x;}
		void set_n(float x) {n=x;}
		void set_c(int i, float x) {c[i]=x;}
		void set_sigma(int i, float x) {sigma[i]=x;}
		void set_w(int i, std::vector<float> x) {w[i]=x;}
		void set_obstacle(vector<float>& o, vector<float>& p);
		void set_y(std::vector<float> input) {y=input;}
		void set_tracking(std::vector<float> k) {trackingPoints = k;}
		void scan_vector_field();
		std::vector<float> get_f() {return f;}
		float get_v() {return v;}
		std::vector<float> get_r() {return r;}
		std::vector<float> get_y();
		std::vector<float> get_z() {return z;}
		float get_w(int i, int j) {return w[i][j];}
		void reset();
		void init_dmp(int dim, std::vector<float> start, std::vector<float> goal, float total_t, float delta_t, float temp_scaling, int n_kernels, float width);
		vector<float> avoid_obstacle_other_way( vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance, float tau);
		vector<float> avoid_obstacle( vector<float> obstacle, vector<float> mobilePoint,  vector<float> speed, float distance, float tau);
		// void set_weights_from_file(const char* file_name);
		void calculate_one_step_dmp(float time_step);
		float repulsive_field_value(vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance);
		vector<float> gradient(vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance, float tau);
		vector<float> mat_gradient(vector<float> obstacle, vector<float> mobilePoint, vector<float> speed, float distance, float tau);
		vector<float> secondary_avoidance(std::vector<float> par_y);
	float LAMBDA;
	float BETA;
	float motion_persistence_weight;
	vector<float> dy;
	vector<float> dz_previous;
	vector<float> persistent_deviation;
	vector<float> persistent_direction_of_motion;
	vector<float> deviation;
	vector<float> persistentDeviationChange;
		std::vector<float> obstacle;
	FILE * positionAndDistanceOutput;
	FILE * deviationOutput;
	FILE * cosAngles;
	FILE * gradientValues;
	FILE * speedAndAcceleration;
	FILE * obstacleList;
	FILE * persistenceOutput;
	FILE * positionAndMotion;
	FILE * verboseAvoidance;
	FILE * goalfunction;
	FILE * verbose;
	FILE * forces;
	FILE * internaldistances;
	FILE * angleanddistance;
      int timestep;
      std::vector<float> s;
      std::vector<float> g;
   public:
      int dimensions;
      float T;
      float dt;
      float tau;
      int n;
      std::vector<float> c;
      std::vector<float> sigma;
      std::vector<std::vector<float> > w;
      float alpha_z;
      float beta_z;
      float alpha_v;
      float alpha_w;
      float v;
      std::vector<float> r;
      std::vector<float> f;
      std::vector<float> y;
      std::vector<float> z;
      std::vector<float> trackingPoints;
      std::vector<float> distances;

};

