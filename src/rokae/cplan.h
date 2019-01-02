#ifndef ROKAE_CPLAN_H_
#define ROKAE_CPLAN_H_

#include <aris.h>

using namespace aris::plan;

struct MoveCParam
{
	int total_time;
	int left_time;
	//double step_size;定义步长参数
	double radius; //圆形规划半径
	double detal;//圆弧轨迹增量
};
class MoveCircle : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveCParam p;
		p.total_time = std::stoi(params.at("total_time"));
		//p.step_size = std::stod(params.at("step_size"));
		p.radius = std::stod(params.at("radius"));
        p.detal = target.model->calculator().calculateExpression(params.at("detal")).toDouble();
		p.left_time = 0;
		target.param = p;
		target.option =
			//用这段话可以不用将model的轨迹赋值到controller里面，系统直接调用model中的反解计算结果，如果
			//不用这个命令，那么需要用for循环将model中的反解计算结果赋值到controller里面
			aris::plan::Plan::USE_TARGET_POS |
            aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;

		
	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
		auto &p = std::any_cast<MoveCParam&>(target.param);

		//static double beginpos[6] = { 0, 0, 0, 0, 0, 0 };
		//if (target.count == 1)
		//{
		//	target.master->mout() << "mot1:" << controller->motionPool()[5].actualPos() << std::endl;
		//	//target.master->mout() << "mot2:" << controller->motionPool()[1].actualPos() << std::endl;

		//	beginpos[5] = controller->motionPool()[5].actualPos();
		//	//beginpos[1] = controller->motionPool()[1].actualPos();

		//	//double pq[7]{ 0,0,0,0,0,0,1 };

		//	
		//}

		static double beginpq[7];
		if (target.count == 1)
		{
			//获取当前模型的位置
			target.model->generalMotionPool()[0].getMpq(beginpq);
		}
		//定义一个变量，四元数，前三个代表位置
		double pq[7];
		//将获取的机器人位置赋值给变量
		std::copy_n(beginpq, 7, pq);
		//对变量的第一个参数进行运动规划
        //pq[1] = beginpq[1] + p.radius*(std::cos(-aris::PI/2+1.0*target.count / p.total_time * 5 * 2 * aris::PI)) + p.detal * target.count;//Y轴，水平轴,走完5个圆
        //pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI/2+1.0*target.count / p.total_time * 5 * 2 * aris::PI)) + p.radius;//Z轴竖直轴
        pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI/2+1.0*target.count / p.total_time * 5 * 2 * aris::PI)) + p.radius;//Z轴竖直轴

        if(target.count %500 ==0)target.master->mout()<< pq[1] <<"  "<<pq[2]<< std::endl;




        //pq[2] = beginpq[2] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
		//将变量的值赋值给model中模型的末端位置
        target.model->generalMotionPool()[0].setMpq(pq);
        //target.model->generalMotionPool()[2].setMpq(pq);
		//求运动学反解许哟啊调用求解器solverpool，kinPos是位置反解，kinVel是速度反解
        if(target.model->solverPool()[0].kinPos() == 0 && target.count %500 ==0)target.master->mout()<< "kin failed"<<std::endl;
        //target.model->solverPool()[2].kinPos();


        //for(int i = 0; i<6;++i)
        //{
         //   controller->motionPool()[i].setTargetPos(target.model->motionPool()[i].mp());
        //}

		//controller->motionPool()[5].setTargetPos(beginpos[5] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI))); //0.01
		//controller->motionPool()[0].setTargetPos(beginpos[0] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI))); //0.01
		//controller->motionPool()[1].setTargetPos(beginpos[1] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 2 * aris::PI)));

		return p.total_time - target.count;
	}
	auto virtual collectNrt(PlanTarget &target)->void {}
	explicit MoveCircle(const std::string &name = "myplan") :Plan(name)
	{
		command().loadXmlStr(
            "<mvEE>"
			"	<group type=\"GroupParam\">"
			"	    <total_time type=\"Param\" default=\"5000\"/>" //默认5000
            "       <radius type=\"Param\" default=\"0.01\"/>"
			"       <detal type=\"Param\" default=\"0.1/5000\"/>"//5秒走10cm
			"   </group>"
            "</mvEE>");
	}
};

#endif
