#include<iostream>
#include<aris.h>

using namespace std;
using namespace aris::plan;

auto createControllerRokaeXB4()->std::unique_ptr<aris::control::Controller>	/*函数返回的是一个类指针，指针指向Controller,controller的类型是智能指针std::unique_ptr*/
{
	std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);/*创建std::unique_ptr实例*/

	std::string xml_str =
		"<m_servo_press type=\"EthercatMotion\" phy_id=\"0\" product_code=\"0x60380007\""
		" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
		" min_pos=\"0.01\" max_pos=\"0.26\" max_vel=\"0.125\" min_vel=\"-0.125\""
		" max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
		" home_pos=\"0\" pos_factor=\"-3355443200\" pos_offset=\"0.0\">"
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
		"				<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"				<offset_tor index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
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
		"</m_servo_press>";
	controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);

	std::string xml_str1 =
		"<m_servo_press type=\"EthercatMotion\" phy_id=\"1\" product_code=\"0x60380007\""
		" vendor_id=\"0x0000066F\" revision_num=\"0x00010000\" dc_assign_activate=\"0x0300\""
		" min_pos=\"0.01\" max_pos=\"0.26\" max_vel=\"0.125\" min_vel=\"-0.125\""
		" max_acc=\"2.0\" min_acc=\"-2.0\" max_pos_following_error=\"0.005\" max_vel_following_error=\"0.005\""
		" home_pos=\"0\" pos_factor=\"-3355443200\" pos_offset=\"0.0\">"
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
		"				<targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
		"				<offset_tor index=\"0x60B2\" subindex=\"0x00\" size=\"16\"/>"
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
		"</m_servo_press>";
	controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str1);

	return controller;
};


//moveJS中的s是指sin函数
struct MoveJSParam
{
	int total_time;
	int left_time;
};

class MoveJS : public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveJSParam p;
		p.total_time = std::stoi(params.at("total_time"));
		p.left_time = 0;
		target.param = p;
		target.option |= USE_TARGET_POS;
	}
	auto virtual executeRT(PlanTarget &target)->int
	{
		MoveJSParam param = std::any_cast<MoveJSParam>(target.param);
		static double begin_pe[6]{ 0,0,0,0,0,0 };
		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].getMpe(begin_pe, "321");
		}
		double pe[6];
		std::copy_n(begin_pe, 6, pe);
		pe[0] = begin_pe[0] + 0.05*(1 - std::cos(1.0*target.count / param.total_time * 2 * aris::PI));
		target.model->generalMotionPool()[0].setMpe(pe, "321");
		target.model->solverPool()[0].kinPos();
		if (target.count % 100 == 0)aris::dynamic::dsp(1, 6, pe);
		return param.total_time - target.count;
	}
	auto virtual collectNrt(PlanTarget &target)->void {}
	explicit MoveJS(const std::string &name = "myplan") :Plan(name)
	{
		command().loadXmlStr(
			"<myplan>"
			"	<total_time type=\"Param\" default=\"209105216\"/>" //默认5000
			"</myplan>");
	}
};

auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
{
	std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

	plan_root->planPool().add<aris::plan::EnablePlan>();
	plan_root->planPool().add<aris::plan::DisablePlan>();
	plan_root->planPool().add<aris::plan::ModePlan>();
	plan_root->planPool().add<aris::plan::RecoverPlan>();
	plan_root->planPool().add<aris::plan::SleepPlan>();
	auto &rs = plan_root->planPool().add<aris::plan::ResetPlan>();
	rs.command().findByName("group")->findByName("pos")->loadXmlStr("<pos default=\"{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5,0.01}\" abbreviation=\"p\"/>");

	plan_root->planPool().add<aris::plan::MovePlan>();
	plan_root->planPool().add<aris::plan::MoveJ>();
	plan_root->planPool().add<MoveJS>();
	plan_root->planPool().add<aris::plan::Show>();

	/*	auto &dm1 = plan_root->planPool().add<aris::plan::MoveJ>();
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.444,-0,0.562,0.642890516,0.000011540,0.765958083,-0.000008196}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,0.334,0.032,-0.018301280,-0.327458252,0.944444310,-0.021473281}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,0.334,0.398,-0.018332796,-0.327460720,0.944442425,-0.021491655}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,-0.344,0.390,-0.025825354,-0.327485510,0.944191478,-0.024264042}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,-0.344,0.085,-0.025828364,-0.327501842,0.944186337,-0.024240465}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.406,-0.344,0.272,-0.025848482,-0.327498467,0.944187605,-0.024215228}\"/>");
		dm1.command().findByName("group")->findByName("unique_pos")->findByName("pq")->loadXmlStr("<pq default=\"{0.444,-0,0.562,0.642890516,0.000011540,0.765958083,-0.000008196}\"/>");*/
	return plan_root;
}

int main(int argc, char *argv[])
{
	auto&cs = aris::server::ControlServer::instance();
	//cs.resetController(aris::robot::createControllerRokaeXB4().release());
	//cs.resetModel(aris::robot::createModelRokaeXB4().release());
	//cs.resetPlanRoot(aris::robot::createPlanRootRokaeXB4().release());
	cs.planRoot().planPool().add<MoveJS>();
	cs.planRoot().planPool().add<MoveJS>();
	cs.start();
	//等待查询
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			auto id = cs.executeCmd(aris::core::Msg(command_in));
			std::cout << "command id:" << id << std::endl;
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			//LOG_ERROR << e.what() << std::endl;
		}
	}

	char ch;
	std::cin >> ch;
}