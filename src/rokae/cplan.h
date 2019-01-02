#ifndef ROKAE_CPLAN_H_
#define ROKAE_CPLAN_H_

#include <aris.h>

using namespace aris::plan;

struct MoveCParam
{
	int total_time;
	int left_time;
	//double step_size;���岽������
	double radius; //Բ�ι滮�뾶
	double detal;//Բ���켣����
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
			//����λ����Բ��ý�model�Ĺ켣��ֵ��controller���棬ϵͳֱ�ӵ���model�еķ�������������
			//������������ô��Ҫ��forѭ����model�еķ����������ֵ��controller����
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
			//��ȡ��ǰģ�͵�λ��
			target.model->generalMotionPool()[0].getMpq(beginpq);
		}
		//����һ����������Ԫ����ǰ��������λ��
		double pq[7];
		//����ȡ�Ļ�����λ�ø�ֵ������
		std::copy_n(beginpq, 7, pq);
		//�Ա����ĵ�һ�����������˶��滮
        //pq[1] = beginpq[1] + p.radius*(std::cos(-aris::PI/2+1.0*target.count / p.total_time * 5 * 2 * aris::PI)) + p.detal * target.count;//Y�ᣬˮƽ��,����5��Բ
        //pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI/2+1.0*target.count / p.total_time * 5 * 2 * aris::PI)) + p.radius;//Z����ֱ��
        pq[2] = beginpq[2] + p.radius*(std::sin(-aris::PI/2+1.0*target.count / p.total_time * 5 * 2 * aris::PI)) + p.radius;//Z����ֱ��

        if(target.count %500 ==0)target.master->mout()<< pq[1] <<"  "<<pq[2]<< std::endl;




        //pq[2] = beginpq[2] + p.step_size*(1 - std::cos(1.0*target.count / p.total_time * 1 * aris::PI));
		//��������ֵ��ֵ��model��ģ�͵�ĩ��λ��
        target.model->generalMotionPool()[0].setMpq(pq);
        //target.model->generalMotionPool()[2].setMpq(pq);
		//���˶�ѧ������Ӵ�����������solverpool��kinPos��λ�÷��⣬kinVel���ٶȷ���
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
			"	    <total_time type=\"Param\" default=\"5000\"/>" //Ĭ��5000
            "       <radius type=\"Param\" default=\"0.01\"/>"
			"       <detal type=\"Param\" default=\"0.1/5000\"/>"//5����10cm
			"   </group>"
            "</mvEE>");
	}
};

#endif
