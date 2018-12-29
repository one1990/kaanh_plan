#ifndef ROKAE_PLAN_H_
#define ROKAE_PLAN_H_

#include <aris.h>

using namespace aris::plan;

struct MoveJSParam
{
	int total_time;
	int left_time;
	double step_size;//
};
class MoveJS : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveJSParam p;
		p.total_time = std::stoi(params.at("total_time"));
		p.step_size = std::stod(params.at("step_size"));//
		p.left_time = 0;
		target.param = p;
		target.option =
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
		auto &p = std::any_cast<MoveJSParam&>(target.param);

		static double beginpos[2];
		if (target.count == 1)
		{
			target.master->mout() << "mot1:" << controller->motionPool()[0].actualPos() << std::endl;
			target.master->mout() << "mot2:" << controller->motionPool()[1].actualPos() << std::endl;

			beginpos[0] = controller->motionPool()[0].actualPos();
			beginpos[1] = controller->motionPool()[1].actualPos();
		}



		controller->motionPool()[0].setTargetPos(beginpos[0] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI))); //0.01
		controller->motionPool()[1].setTargetPos(beginpos[1] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI)));

		return p.total_time - target.count;
	}
	auto virtual collectNrt(PlanTarget &target)->void {}
	explicit MoveJS(const std::string &name = "myplan") :Plan(name)
	{
		command().loadXmlStr(
			"<myplan>"
			"	<group type=\"GroupParam\">"
			"	    <total_time type=\"Param\" default=\"5000\"/>" //д╛хо5000
			"       <step_size type=\"Param\" default=\"0.045\"/>"
			"   </group>"
			"</myplan>");
	}
};




#endif