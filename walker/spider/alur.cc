#include <mirmik.h>

gazebo::TrotController::TrotController(BodyController * body_controller) :
	body_controller()
{
	active_group = &group0;

	group0.push_back(&body_controller->legs[0]);
	group0.push_back(&body_controller->legs[2]);
	group0.push_back(&body_controller->legs[4]);

	group1.push_back(&body_controller->legs[1]);
	group1.push_back(&body_controller->legs[3]);
	group1.push_back(&body_controller->legs[5]);
};

bool gazebo::TrotController::evaluate_active_group()
{
	return false;
}

void gazebo::TrotController::serve(double delta)
{
	evaluate_active_group();

	for (auto l : *active_group)
	{
		l->SpeedControl(-body_controller->target_body_speed);
		l->serve(delta);
	}

	for (auto l : *relax_group)
	{
		if (stage == 0)
		{
			l->SpeedControl({0, 0, 0});
		}
		else if (stage == 0)
		{
			l->SpeedControl({0, 0, 0});
		}
		else if (stage == 0)
		{
			l->SpeedControl({0, 0, 0});
		}

		l->serve(delta);
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

		l->serve(delta);
	}
}