#include <algorithm>
#include"rokae.h"
#include "plan.h"
#include "cplan.h"

using namespace aris;
using namespace aris::dynamic;
using namespace aris::plan;

namespace rokae
{
	auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*�������ص���һ����ָ�룬ָ��ָ��Controller,controller������������ָ��std::unique_ptr*/
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*����std::unique_ptrʵ��*/

		for (aris::Size i = 0; i < 6; ++i)
		{

#ifdef WIN32
			double pos_offset[6]
			{
				0,   0,	  0,   0,   0,   0
			};
#endif
#ifdef UNIX
			double pos_offset[6]
			{
				0.00293480352126769,0.317555328381088,-0.292382537944081,0.0582675097338009,1.53363576057128,17.1269434336436
			};
#endif
			double pos_factor[6]
			{
				131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 72.857 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 50 / 2 / PI
			};
			double max_pos[6]
			{
				170.0 / 360 * 2 * PI, 130.0 / 360 * 2 * PI,	50.0 / 360 * 2 * PI, 170.0 / 360 * 2 * PI, 117.0 / 360 * 2 * PI, 360.0 / 360 * 2 * PI,
			};
			double min_pos[6]
			{
				-170.0 / 360 * 2 * PI, -84.0 / 360 * 2 * PI, -188.0 / 360 * 2 * PI, -170.0 / 360 * 2 * PI, -117.0 / 360 * 2 * PI, -360.0 / 360 * 2 * PI
			};
			double max_vel[6]
			{
				310.0 / 360 * 2 * PI, 240.0 / 360 * 2 * PI, 310.0 / 360 * 2 * PI, 250.0 / 360 * 2 * PI, 295.0 / 360 * 2 * PI, 500.0 / 360 * 2 * PI,
			};
			double max_acc[6]
			{
				1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 1750.0 / 360 * 2 * PI, 1500.0 / 360 * 2 * PI, 2500.0 / 360 * 2 * PI,
			};

			std::string xml_str =
                "<m" + std::to_string(i) + " type=\"EthercatMotion\" phy_id=\"" + std::to_string(i) + "\" product_code=\"0x0\""
				" vendor_id=\"0x000002E1\" revision_num=\"0x29001\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.1\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
				"	<sm_pool type=\"SyncManagerPoolObject\">"
				"		<sm type=\"SyncManager\" is_tx=\"false\"/>"
				"		<sm type=\"SyncManager\" is_tx=\"true\"/>"
				"		<sm type=\"SyncManager\" is_tx=\"false\">"
				"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"false\">"
				"				<control_word index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
				"				<mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"				<target_pos index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
				"				<target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
				"				<offset_vel index=\"0x60B1\" subindex=\"0x00\" size=\"32\"/>"
				"				<target_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
				"			</index_1600>"
				"		</sm>"
				"		<sm type=\"SyncManager\" is_tx=\"true\">"
				"			<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
				"				<status_word index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
				"				<mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
				"				<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
				"				<vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
				"				<cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"16\"/>"
				"			</index_1a00>"
				"		</sm>"
				"	</sm_pool>"
				"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
				"	</sdo_pool>"
				"</m" + std::to_string(i) + ">";

			controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
		}

		return controller;
	};
	auto createModelRokaeXB4(const double *robot_pm)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// �������� //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// ��ӱ��� //
		model->calculator().addVariable("PI", aris::core::Matrix(aris::PI));

		// add part //
		auto &p1 = model->partPool().add<Part>("L1");
		auto &p2 = model->partPool().add<Part>("L2");
		auto &p3 = model->partPool().add<Part>("L3");
		auto &p4 = model->partPool().add<Part>("L4");
		auto &p5 = model->partPool().add<Part>("L5");
		auto &p6 = model->partPool().add<Part>("L6");

		// add joint //
		const double j1_pos[3]{ 0.0, 0.0, 0.176 };
		const double j2_pos[3]{ 0.04, -0.0465, 0.3295, };
		const double j3_pos[3]{ 0.04, 0.0508, 0.6045 };
		const double j4_pos[3]{ -0.1233, 0.0, 0.6295, };
		const double j5_pos[3]{ 0.32, -0.03235, 0.6295, };
		const double j6_pos[3]{ 0.383, 0.0, 0.6295, };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 1.0, 0.0, 0.0 };
		const double j5_axis[6]{ 0.0, 1.0, 0.0 };
		const double j6_axis[6]{ 1.0, 0.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);

		// add ee general motion //
		double pq_ee_i[]{ 0.398, 0.0, 0.6295, 0.0, 0.0, 0.0, 1.0 };
		double pm_ee_i[16];
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee_i);
		auto &makJ = model->ground().markerPool().add<Marker>("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// change robot pose //
		if (robot_pm)
		{
			p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
			p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
			p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
			p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
			p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
			p6.setPm(s_pm_dot_pm(robot_pm, *p6.pm()));
			j1.makJ().setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ().prtPm()));
		}

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PumaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();

		inverse_kinematic.setWhichRoot(8);

		return model;
	}
	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
		std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::EnablePlan>();
		plan_root->planPool().add<aris::plan::DisablePlan>();
		plan_root->planPool().add<aris::plan::ModePlan>();
		plan_root->planPool().add<aris::plan::RecoverPlan>();
		plan_root->planPool().add<aris::plan::SleepPlan>();
		auto &rs = plan_root->planPool().add<aris::plan::ResetPlan>();
        rs.command().findByName("group")->findByName("pos")->loadXmlStr("<pos default=\"{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}\" abbreviation=\"p\"/>");

		plan_root->planPool().add<aris::plan::MovePlan>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<MoveJS>();//��ӵ���һ����
		plan_root->planPool().add<MoveCircle>();
		plan_root->planPool().add<MoveTroute>();
		plan_root->planPool().add<MoveFile>();
		return plan_root;
	}
}
