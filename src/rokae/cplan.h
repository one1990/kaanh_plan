#ifndef ROKAE_CPLAN_H_
#define ROKAE_CPLAN_H_

#include <aris.h>
using namespace aris::plan;

struct MoveCParam
{
	int total_time;
	int left_time;
	double radius; //Բ�ι滮�뾶
	double detal;//Բ���켣����
    double theta;//������м����
};

class MoveCircle : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
        MoveCParam p;
		p.total_time = std::stoi(params.at("total_time"));
		p.radius = std::stod(params.at("radius"));
        p.detal = target.model->calculator().calculateExpression(params.at("detal")).toDouble();//���ֱ����stod����detal�г���5000�ķ�ĸֱ�ӱ�������
		p.left_time = 0;
		target.param = p;
		target.option =
			//����λ����Բ��ý�model�Ĺ켣��ֵ��controller���棬ϵͳֱ�ӵ���model�еķ�������������
			//������������ô��Ҫ��forѭ����model�еķ����������ֵ��controller����
			aris::plan::Plan::USE_TARGET_POS |
            aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | //��ʼ������ٶ�����
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;		
	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
		auto &p = std::any_cast<MoveCParam&>(target.param);

		static double beginpq[7];
		if (target.count == 1)
		{
			//��ȡ��ǰģ�͵�λ��
			target.model->generalMotionPool()[0].getMpq(beginpq);
		}
		//����һ����������Ԫ����ǰ��������λ��
		double pq[7];
		//����ȡ�Ļ�����λ�ø�ֵ������
		std::copy_n(beginpq, 7, pq);
		//�Ա����ĵ�һ�����������˶��滮
		//��theta����Ϊ�ٶ������ģ���Ϊ�����ҡ������󵼺�Ҫ���ԽǶȵ��󵼣��Ƕȵ��󵼳�ʼֵΪ0��Ϊ�ٶ�����
        double theta = 1.0*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
        pq[1] = beginpq[1] + p.radius*(std::cos(-aris::PI/2+1.0* theta * 5 * 2 * aris::PI)) + p.detal * theta * 5000;//Y�ᣬˮƽ��,����5��Բ
        pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI/2+1.0*theta * 5 * 2 * aris::PI)) + p.radius;//Z����ֱ��
        if(target.count %500 ==0)target.master->mout()<< pq[1] <<"  "<<pq[2]<< std::endl;
		//��������ֵ��ֵ��model��ģ�͵�ĩ��λ��
        target.model->generalMotionPool()[0].setMpq(pq);
        //target.model->generalMotionPool()[2].setMpq(pq);
		//���˶�ѧ������Ҫ���������solverpool��kinPos��λ�÷��⣬kinVel���ٶȷ���
        if(target.model->solverPool()[0].kinPos() == 0 && target.count %500 ==0)target.master->mout()<< "kin failed"<<std::endl;
        //ֻ��һ��ĩ��λ�ã�����ֻ��һ�������������������䲻��
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
			"	    <total_time type=\"Param\" default=\"5000\"/>" //Ĭ��5000
            "       <radius type=\"Param\" default=\"0.01\"/>"
			"       <detal type=\"Param\" default=\"0.1/5000\"/>"//5����10cm
			"   </group>"
            "</mvEE>");
	}
};
//ĩ�˿ռ�����ι켣����

struct MoveTParam
{
	int total_time;
	double vel;
	double acc;
	double dec;//v0�����ι켣�����ٶ�
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
			//����λ����Բ��ý�model�Ĺ켣��ֵ��controller���棬ϵͳֱ�ӵ���model�еķ�������������
			//������������ô��Ҫ��forѭ����model�еķ����������ֵ��controller����
			aris::plan::Plan::USE_TARGET_POS |
			aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | //��ʼ������ٶ�����
			aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
	}
	auto virtual  executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
		auto &p = std::any_cast<MoveTParam&>(target.param);
		double pt, v, a;
		aris::Size t_count;
		aris::Size total_count=1;
		double begin_pos=0.0;//�ֲ�������ø�һ����ʼֵ
		if (target.count == 1)
		{
			//target.model->generalMotionPool()[0].getMpq(beginpq);
			begin_pos = controller->motionAtAbs(5).targetPos();
		}

		//double pq[7];

		//std::copy_n(beginpq, 7, pq);
		aris::plan::moveAbsolute(target.count, begin_pos ,p.pt , p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, pt, v, a, t_count);
		//aris::plan::moveAbsolute(target.count, param.begin_pos, param.pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);
		target.model->motionPool().at(5).setMp(pt);//motionpool�������ĳ��ӡ�����
		//target.model->motionPool().at(5).setMv(v * 1000);
		total_count = std::max(total_count, t_count);
		
		
		//double theta = 1.0*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
		//pq[0] = beginpq[0] + p.radius*(std::cos(-aris::PI / 2 + 1.0* theta * 5 * 2 * aris::PI)) + p.detal * theta * 5000;//Y�ᣬˮƽ��,����5��Բ
		//pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI / 2 + 1.0*theta * 5 * 2 * aris::PI)) + p.radius;//Z����ֱ��
		//if (target.count % 500 == 0)target.master->mout() << pq[1] << "  " << pq[2] << std::endl;
		//��������ֵ��ֵ��model��ģ�͵�ĩ��λ��
		//target.model->generalMotionPool()[0].setMpq(pq);
		//target.model->generalMotionPool()[2].setMpq(pq);
		//���˶�ѧ������Ҫ���������solverpool��kinPos��λ�÷��⣬kinVel���ٶȷ���
		if (target.model->solverPool()[0].kinPos() == 0 && target.count % 500 == 0)target.master->mout() << "kin failed" << std::endl;
		//ֻ��һ��ĩ��λ�ã�����ֻ��һ�������������������䲻��
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
			"	    <total_time type=\"Param\" default=\"5000\"/>" //Ĭ��5000
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
