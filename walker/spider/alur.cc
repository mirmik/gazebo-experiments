#include <mirmik.h>

gazebo::TrotController::TrotController(BodyController * body_controller) :
	body_controller(body_controller)
{
};

void gazebo::TrotController::init() 
{
	group0.push_back(&body_controller->legs[0]);
	group0.push_back(&body_controller->legs[2]);
	group0.push_back(&body_controller->legs[4]);

	group1.push_back(&body_controller->legs[1]);
	group1.push_back(&body_controller->legs[3]);
	group1.push_back(&body_controller->legs[5]);	

	active_group = group0;
	relax_group = group1;
}

void gazebo::TrotController::serve(double delta)
{
	if (iteration_start) 
	{
		iteration_start = false;
		start_iteration = evaltime();
	}

	//body_controller->body_target = {{0,0,0,1},{0,0,1}};
	body_controller->body_serve_with_group(active_group, delta);

	if (relax_end_step)
	{
		bool all_landed = true;
		for (auto * ptr : relax_group)
		{
			bool not_landed = !ptr->is_landed();
			if (not_landed)
			{
				all_landed = false;

				ptr->speed_target = {0, 0, -0.1};
				ptr->serve(delta, SpeedMode, SpeedMode);
			}
		}

		if (all_landed)
		{
			change_active_group();
			relax_end_step = false;
		}
	}
	else
	{
		for (auto * ptr : relax_group)
		{
			auto stress_factor = 0;

			if (stress_factor > 0.3)
				relax_end_step = true;

			body_controller->relax_movement_with_group(
			    relax_group, delta, 0.1,
			    - body_controller->body_speed_target.lin * 2);
		}

		if (evaltime() - start_iteration) 
		{
			relax_end_step = false;
		}
	}
}

gazebo::StepController::StepController(BodyController * body_controller) :
	body_controller(body_controller)
{}


void gazebo::StepController::evaluate_active()
{

}

void gazebo::StepController::serve(double delta)
{
	evaluate_active();

	for (int i = 0; i < body_controller->legs.size(); i++)
	{
		auto l = &body_controller->legs[i];

		if (i != relax_leg)
		{
			l->PositionControl(l->relax_pose
			                   + linalg::vec<double, 3> {10, 0, 0});
		}
		else
		{
			if (l->is_landed())
			{
				l->SpeedControl({0, 0, -0.5});
			}
			else
			{
			}
		}

		l->serve(delta, SpeedMode, SpeedMode);
	}
}

void gazebo::TrotController::change_active_group() 
{
	std::swap(active_group, relax_group);

}