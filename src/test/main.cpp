#include <iostream>
#include <fstream>
#include <typeinfo>
using namespace std;
static double POS[59815][72];
int main()
{
	//¶ÁÈ¡txtÎÄ¼þ
	int m = 59815;
	int n = 72;
	//n = 18;
	
	std::fill_n(*POS, 59815 * 72, 0.0);

	static ifstream oplog;
	oplog.open("C:\\Users\\qianch_kaanh_cn\\Desktop\\myplan\\src\\rokae\\rt.txt");
	for (int i = 0; i < m; i++)
	{
		if (i > 2500 && i < 2600)
		{
			cout << "line "<<i<<":";
		}
		
		for (int j = 0; j < n; j++)
		{
			oplog >> POS[i][j];
			if (oplog.bad())
			{
				std::cout << "bad" << std::endl;
			}

			if (i > 2500 && i < 2600)
			{
				cout << POS[i][j]<<"  ";
			}

		}

		if (i > 2500 && i < 2600)
		{
			cout << endl;
		}
	}
	oplog.close();
	float P1[59815];
	for (int i = 0; i < 59815; i++)
	{
		int j = 1;
		P1[i] = POS[i][j];
	}
	/*cout << "P1[0]" << ':' << P1[0] << endl;
	cout << "P1[1]" << ':' << P1[1] << endl;
	cout << "P1[3]" << ':' << P1[3] << endl;
	cout << "P1[1000]" << ':' << P1[1000] << endl;
	cout << "P1[59810]" << ':' << P1[59810] << endl;
	cout << "P1[59814]" << ':' << P1[59814] << endl;*/
	cout << "P[5000][0]" << ':' << POS[5000][0] << endl;

	cout << "P[0][0]" << ':' << POS[0][0] << endl;
	cout << "P[0][74]" << ':' << POS[0][74] << endl;
	cout << "P[0][1]" << ':' << POS[0][1] << endl;
	cout << "P[0][72]" << ':' << POS[0][72] << endl;
	cout << "P[0][3]" << ':' << POS[0][3] << endl;
	cout << "P[0][4]" << ':' << POS[0][4] << endl;
	cout << "P[0][12]" << ':' << POS[0][12] << endl;
	cout << "P[1][1]" << ':' << POS[1][1] << endl;
	cout << "P[1][3]" << ':' << POS[1][3] << endl;
	cout << "P[3][3]" << ':' << POS[3][3] << endl;
	cout << "P[55][2]" << ':' << POS[55][2] << endl;

}


