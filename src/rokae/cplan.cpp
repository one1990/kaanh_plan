#include <aris.h>
#include <iostream>
#include <fstream>
#include <typeinfo>
#include <vector>
#include "cplan.h"
using namespace std;
using namespace aris::plan;

/// \brief
struct MoveCParam
{
    int total_time;
    int left_time;
    double radius; //Բ�ι滮�뾶
    double detal;//Բ���켣����
    double theta;//������м����
};


auto MoveCircle::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveCParam p;
    p.total_time = std::stoi(params.at("total_time"));
    p.radius = std::stod(params.at("radius"));
    p.detal = target.model->calculator().calculateExpression(params.at("detal")).toDouble();//���ֱ����stod����detal�г���5000�ķ�ĸֱ�ӱ�������
    p.left_time = 0;
    target.param = p;
    target.option =
        //����λ����Բ��ý�model�Ĺ켣��ֵ��controller���棬ϵͳֱ�ӵ���model�еķ��������
        aris::plan::Plan::USE_TARGET_POS |
        aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;
}
auto MoveCircle::executeRT(PlanTarget &target)->int
{
    auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
    auto &p = std::any_cast<MoveCParam&>(target.param);

    static double beginpq[7];
    if (target.count == 1)
    {
        //��ȡ��ǰģ�͵�λ��
        target.model->generalMotionPool()[0].getMpq(beginpq);//generalMotionPool()[0]һ��ĩ��
    }
    //����һ����������Ԫ����ǰ��������λ��
    double pq[7];
    //����ȡ�Ļ�����λ�ø�ֵ������
    std::copy_n(beginpq, 7, pq);
    //�Ա����ĵ�һ�����������˶��滮
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
    return p.total_time - target.count;
}
//auto MoveCircle::collectNrt(PlanTarget &target)->void {}
MoveCircle::MoveCircle(const std::string &name) :Plan(name)
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


//ĩ�˿ռ�����ι켣����
struct MoveTParam
{
    int total_time;
    double vel;
    double acc;
    double dec;//v0�����ι켣�����ٶ�
    double pt;
};

    auto MoveTroute::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
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
    auto MoveTroute::executeRT(PlanTarget &target)->int
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
        aris::plan::moveAbsolute(target.count, begin_pos ,p.pt , p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, pt, v, a, t_count);
        target.model->motionPool().at(5).setMp(pt);//motionpool�������ĳ��ӡ�����
        //target.model->motionPool().at(5).setMv(v * 1000);
        total_count = std::max(total_count, t_count);


        //if (target.model->solverPool()[0].kinPos() == 0 && target.count % 500 == 0)target.master->mout() << "kin failed" << std::endl;

        return p.total_time - target.count;
    }
    MoveTroute::MoveTroute(const std::string &name):Plan(name)
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


/// \brief �ṹ������
///�ṹ��MoveFileParam�� ���ڶ�ȡ.txt�ļ��ĵ�λ����Ϣ�������ƻ����˰����ļ��е�λ���˶�
struct MoveFileParam
{
    int total_time;
    int m, n;  // m,n����txt�ĵ������ݵ�����������
    double vel;
    double acc;
    double dec;// v0�����ι켣�����ٶ�
    vector<double> pt;
    int choose;
};
//vector<vector<double>  > POS(72, std::vector<double>(700, 0.0));
/// ������ά�������ڴ洢�ļ��е�λ����Ϣ
//static double POS[59815][72];
//static double P1[59815];
//vector <double> P1;
vector<vector<double>  > POS(18);
/// \brief ��MoveFile����
/// ����MoveFile��,��ɶ����е�.txt�ļ��е�λ�����ݵ���ȡ��ͨ��������ƻ����˵ĸ��ؽڰ�������λ�ñ仯���˶�
/// ### ��MoveFile
/// + ʵʱ�˵�׼������prepairNrt
/// + ʵʱ�˺���executeRT
/// + �๹�캯��MoveFileʵʱ��ȡxml�ļ���Ϣ

/// \brief ʵʱ�˵�׼������prepairNrt
/// @param &params ͷ�ļ�std�е���map������������string��ֵ������Ҳ��string
/// @param PlanTarget �����ռ�aris::plan�ж���Ľṹ�壬target����������
/// @return ����ֵΪ��
auto MoveFile::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
{
    MoveFileParam p;
    p.total_time = std::stoi(params.at("total_time"));
    p.vel = std::stod(params.at("vel"));
    p.acc = std::stod(params.at("acc"));
    p.dec = std::stod(params.at("dec"));
    p.choose= std::stoi(params.at("choose"));
    aris::core::Matrix mat = target.model->calculator().calculateExpression(params.at("pt"));
    p.pt.resize(mat.size());
    std::copy(mat.begin(), mat.end(), p.pt.begin());//begin��end���Ǳ�׼��vector�ĵ�����
    //for(int i=0;i<6;i++)
    //{
    //    std::cout<<p.pt[i]<<", ";
    //}
    //std::cout<<std::endl;

    // ��ȡtxt�ļ�
    p.m = std::stoi(params.at("m"));
    p.n = std::stoi(params.at("n"));
    //std::fill_n(*POS, 59815 * 72, 0.0);

    for (int j = 0; j < p.n; j++)
    {
        POS[j].clear();
    }

    ifstream oplog;
    int cal = 0;
    //oplog.open("C:\\Users\\qianch_kaanh_cn\\Desktop\\myplan\\src\\rokae\\rt_log--2019-01-10--14-29-07--5--1.txt");
    oplog.open("/home/kaanh/Desktop/myplan/src/rokae/1.txt");
    while (!oplog.eof())
    {
        for (int j = 0; j < p.n; j++)
        {
            // oplog >> POS[i][j];
            double data;
            oplog >> data;

            POS[j].push_back(data);
        }
    }

    oplog.close();
    oplog.clear();
    for (int j = 0; j < p.n; j++)
    {
        POS[j].pop_back();
    }
    target.param = p;
    target.option =
        //aris::plan::Plan::USE_TARGET_POS |
        aris::plan::Plan::NOT_CHECK_VEL_MIN |
        aris::plan::Plan::NOT_CHECK_VEL_MAX |
        aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
        aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
        aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR |
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START | // ��ʼ������ٶ�����
        aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS;


}
/// \brief ʵʱ�˺���executeRT
/// @param PlanTarget �����ռ�aris::plan�ж���Ľṹ�壬target����������
/// @return ����ֵΪ��
auto MoveFile::executeRT(PlanTarget &target)->int
{
    auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
    auto &p = std::any_cast<MoveFileParam&>(target.param);
    double ptt, v, a;
    aris::Size t_count;
    aris::Size total_count = 1;
    aris::Size return_value=0;
    static double begin_pos[6] = { 0.0,0.0,0.0,0.0,0.0,0.0 }; // �ֲ�������ø�һ����ʼֵ;

    if (target.count == 1)
    {
        //target.model->generalMotionPool()[0].getMpq(beginpq);
        for (int i = 0; i < 6; i++)
        {
            // �ڵ�һ�����������ι滮��λ
            begin_pos[i] = controller->motionAtAbs(i).actualPos();// ��ȡ6�������ʼλ��
            //target.model->motionPool().at(i).setMp(controller->motionAtAbs(i).actualPos());
            //if (!target.model->solverPool().at(1).kinPos())return -1;
            //begin_pos[i] = target.model->motionPool().at(i).mp();
        }
    }

    if (p.choose == 0)
    {
        for (int i = 0; i < 6; i++)
        {
            // �ڵ�һ�����������ι滮��λ
            aris::plan::moveAbsolute(target.count, begin_pos[i], p.pt[i], p.vel / 1000, p.acc / 1000 / 1000, p.dec / 1000 / 1000, ptt, v, a, t_count);
            //target.model->motionPool().at(i).setMp(ptt);// motionpool�������ĳ��ӡ�����
            //target.model->motionPool().at(5).setMv(v * 1000);
            controller->motionAtAbs(i).setTargetPos(ptt);
            total_count = std::max(total_count, t_count);

        }
        return_value = target.count>total_count ?0:1;
    }

    if (p.choose == 1)
    {
        //target.master->mout() << POS[0][target.count] << std::endl;
        //target.model->motionPool().at(0).setMp(POS[0][target.count]);// motionpool�������ĳ��ӡ�����
        //target.model->motionPool().at(1).setMp(POS[3][target.count]);
        //target.model->motionPool().at(2).setMp(POS[6][target.count]);
        //target.model->motionPool().at(3).setMp(POS[9][target.count]);
        //target.model->motionPool().at(4).setMp(POS[12][target.count]);
        //target.model->motionPool().at(5).setMp(POS[15][target.count]);

        for (int i = 0; i < 6; i++)
        {
            controller->motionAtAbs(i).setTargetPos(POS[3*i][target.count]);
        }
        return_value = target.count>POS[0].size()?0:1;

    }

    //if (!target.model->solverPool().at(1).kinPos())return -1;

    auto &lout = controller->lout();
    for (int i = 0; i < 6;i++)
    {
        lout << POS[3*i][target.count-1] << endl;
    }

    //for (Size i = 0; i < 6; i++)
    //{
    //	lout << controller->motionAtAbs(i).targetPos() << ",";
    //	lout << controller->motionAtAbs(i).actualPos() << ",";
    //	lout << controller->motionAtAbs(i).actualVel() << ",";
    //	lout << controller->motionAtAbs(i).actualCur() << ",";
    //}
    //lout << std::endl;
    auto &cout = controller->mout();
    if (target.count % 500 == 0)
    {
        for (int i = 0; i < 6;i++)
        {
            cout <<"size:"<<POS[0].size()<<"    "<<begin_pos[i]<<"    "<< p.pt[i]<< ",  ";
        }
         cout << std::endl;
        //target.master->mout() << "POS[1].size()"<<POS[0].size() << " "<<POS.size() << "  POS[1].size()  " << std::endl;
    }
    //target.model-`>motionPool().at(5).setMv(p.POS[;][63])
    //target.model->motionPool().at(5).setMv(v * 1000);
    //total_count = std::max(total_count, t_count);
    //���logλ���ļ�

    /*auto &cout = controller->mout();
    for (int i = 0; i < 6; i++)
    {
        cout << POS[3*i][count] << ",";
    }*/

    //if (target.model->solverPool()[0].kinPos() == 0 && target.count % 500 == 0)target.master->mout() << "kin failed" << std::endl;

    //return total_count - target.count;
    return return_value;

}

/// + �๹�캯��MoveFileʵʱ��ȡxml�ļ���Ϣ
///
MoveFile::MoveFile(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<mvFi>"
            "	<group type=\"GroupParam\" default_child_type=\"Param\">"
            "	    <total_time type=\"Param\" default=\"5000\"/>" // Ĭ��5000
            "		<m type=\"Param\" default=\"59815\"/>"  // ����
            "		<n type=\"Param\" default=\"18\"/>"
            "		<vel type=\"Param\" default=\"0.04\"/>"
            "		<acc type=\"Param\" default=\"0.08\"/>"
            "		<dec type=\"Param\" default=\"0.08\"/>"
            "		<choose type=\"Param\" default=\"0\"/>"
            "		<pt type=\"Param\" default=\"{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
            "	</group>"
            "</mvFi>");
    }

