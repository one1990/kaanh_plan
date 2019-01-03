#ifndef ROKAE_CPLAN_H_
#define ROKAE_CPLAN_H_

#include <aris.h>
using namespace aris::plan;

struct MoveCParam
{
	int total_time;
	int left_time;
	double radius; //圆形规划半径
	double detal;//圆弧轨迹增量
    double theta;//计算的中间变量
};

class MoveCircle : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
        MoveCParam p;
		p.total_time = std::stoi(params.at("total_time"));
		p.radius = std::stod(params.at("radius"));
        p.detal = target.model->calculator().calculateExpression(params.at("detal")).toDouble();//如果直接用stod，则detal中除以5000的分母直接被忽略了
		p.left_time = 0;
		target.param = p;
		target.option =
			//用这段话可以不用将model的轨迹赋值到controller里面，系统直接调用model中的反解计算结果，如果
			//不用这个命令，那么需要用for循环将model中的反解计算结果赋值到controller里面
			aris::plan::Plan::USE_TARGET_POS |
            aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | //开始不检查速度连续
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;		
	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
		auto &p = std::any_cast<MoveCParam&>(target.param);

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
		//将theta定义为速度连续的，因为对正弦、余弦求导后，要乘以角度的求导，角度的求导初始值为0即为速度连续
        double theta = 1.0*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
        pq[1] = beginpq[1] + p.radius*(std::cos(-aris::PI/2+1.0* theta * 5 * 2 * aris::PI)) + p.detal * theta * 5000;//Y轴，水平轴,走完5个圆
        pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI/2+1.0*theta * 5 * 2 * aris::PI)) + p.radius;//Z轴竖直轴
        if(target.count %500 ==0)target.master->mout()<< pq[1] <<"  "<<pq[2]<< std::endl;
		//将变量的值赋值给model中模型的末端位置
        target.model->generalMotionPool()[0].setMpq(pq);
        //target.model->generalMotionPool()[2].setMpq(pq);
		//求运动学反解需要调用求解器solverpool，kinPos是位置反解，kinVel是速度反解
        if(target.model->solverPool()[0].kinPos() == 0 && target.count %500 ==0)target.master->mout()<< "kin failed"<<std::endl;
        //只有一个末端位置，所以只有一个求解器，所以以下语句不对
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
	explicit MoveCircle(const std::string &name = "MoveCircle") :Plan(name)
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
//末端空间的梯形轨迹控制

struct MoveTParam
{
	int total_time;
	double vel;
	double acc;
	double dec;//v0是梯形轨迹最终速度
	double pt;
};
class MoveTroute : public aris::plan::Plan
{
public:
	auto virtual  prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveTParam p;
		p.total_time = std::stoi(params.at("total_time"));
		p.vel = std::stod(params.at("vel"));
		p.acc = std::stod(params.at("acc"));
		p.dec = std::stod(params.at("dec"));
		p.pt = std::stod(params.at("pt"));
	
		target.param = p;
		target.option =
			//用这段话可以不用将model的轨迹赋值到controller里面，系统直接调用model中的反解计算结果，如果
			//不用这个命令，那么需要用for循环将model中的反解计算结果赋值到controller里面
			aris::plan::Plan::USE_TARGET_POS |
			aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | //开始不检查速度连续
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
	}
	auto virtual  executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
		auto &p = std::any_cast<MoveTParam&>(target.param);
		double pt, v, a;
		aris::Size t_count;
		aris::Size total_count=1;
		double begin_pos=0.0;//局部变量最好赋一个初始值
		if (target.count == 1)
		{
			//target.model->generalMotionPool()[0].getMpq(beginpq);
			begin_pos = controller->motionAtAbs(5).targetPos();
		}

		//double pq[7];

		//std::copy_n(beginpq, 7, pq);
		aris::plan::moveAbsolute(target.count, begin_pos ,p.pt , p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, pt, v, a, t_count);
		//aris::plan::moveAbsolute(target.count, param.begin_pos, param.pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
		target.model->motionPool().at(5).setMp(pt);//motionpool驱动器的池子、数组
		//target.model->motionPool().at(5).setMv(v * 1000);
		total_count = std::max(total_count, t_count);
		
		
		//double theta = 1.0*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
		//pq[0] = beginpq[0] + p.radius*(std::cos(-aris::PI / 2 + 1.0* theta * 5 * 2 * aris::PI)) + p.detal * theta * 5000;//Y轴，水平轴,走完5个圆
		//pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI / 2 + 1.0*theta * 5 * 2 * aris::PI)) + p.radius;//Z轴竖直轴
		//if (target.count % 500 == 0)target.master->mout() << pq[1] << "  " << pq[2] << std::endl;
		//将变量的值赋值给model中模型的末端位置
		//target.model->generalMotionPool()[0].setMpq(pq);
		//target.model->generalMotionPool()[2].setMpq(pq);
		//求运动学反解需要调用求解器solverpool，kinPos是位置反解，kinVel是速度反解
		if (target.model->solverPool()[0].kinPos() == 0 && target.count % 500 == 0)target.master->mout() << "kin failed" << std::endl;
		//只有一个末端位置，所以只有一个求解器，所以以下语句不对
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
	explicit MoveTroute(const std::string &name = "MoveTroute"):Plan(name)
	{
		command().loadXmlStr(
			"<mvTT>"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"	    <total_time type=\"Param\" default=\"5000\"/>" //默认5000
			"		<pt type=\"Param\" default=\"0.1\"/>"
			"		<vel type=\"Param\" default=\"0.04\"/>"
			"		<acc type=\"Param\" default=\"0.08\"/>"
			"		<dec type=\"Param\" default=\"0.08\"/>"
			"	</group>"
			"</mvTT>");
		//"</moveEAP>");
	}		
};
#endif
