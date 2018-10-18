#pragma warning( disable : 4996) 
//-----------MySQL
//#include "position.h"
#include <stdio.h>
#include <WinSock.h>  //һ��Ҫ�������������winsock2.h
#include "include/mysql.h"    //����mysqlͷ�ļ�(һ�ַ�ʽ����vcĿ¼�������ã�һ�����ļ��п�������Ŀ¼��Ȼ����������)
#include <Windows.h>
//�������������Ҳ�����ڹ���--������������
#pragma comment(lib,"wsock32.lib")
#pragma comment(lib,"libmysql.lib")
MYSQL mysql; //mysql����
MYSQL_FIELD *fd;  //�ֶ�������
char field[32][32];  //���ֶ�����ά����
MYSQL_RES *res; //����ṹ�������е�һ����ѯ�����
MYSQL_ROW column; //һ�������ݵ����Ͱ�ȫ(type-safe)�ı�ʾ����ʾ�����е���
char query[150]; //��ѯ���

bool ConnectDatabase()
{
	//��ʼ��mysql
	mysql_init(&mysql);  //����mysql�����ݿ�
	
						 //����false������ʧ�ܣ�����true�����ӳɹ�
	if (!(mysql_real_connect(&mysql, "192.168.0.104", "root", "root", "STAr", 0, NULL, 0))) //�м�ֱ����������û��������룬���ݿ������˿ںţ�����дĬ��0����3306�ȣ���������д�ɲ����ٴ���ȥ
	{
		printf("Error connecting to database:%s\n", mysql_error(&mysql));
		return false;
	}
	else
	{
		printf("Connected...\n");
		return true;
	}
}

void FreeConnect()
{
	//�ͷ���Դ
	mysql_free_result(res);
	mysql_close(&mysql);
}

void ModifyData(float time, float x, float y)
{
	sprintf(query, "update data set xPosition=%d where id=1", int(x));
	if (mysql_query(&mysql, query))
		printf("Query failed (%s)\n", mysql_error(&mysql));

	sprintf(query, "update data set yPosition=%d where id=1", int(y));
	if (mysql_query(&mysql, query))
		printf("Query failed (%s)\n", mysql_error(&mysql));

	sprintf(query, "update data set time=%f where id=1", time);
	if (mysql_query(&mysql, query))
		printf("Query failed (%s)\n", mysql_error(&mysql));
}
//
//bool QueryDatabase1()
//{
//	sprintf(query, "select rudder from data where id=1"); //ִ�в�ѯ��䣬�����ǲ�ѯ���У�user�Ǳ��������ü����ţ���strcpyҲ����
//														  //mysql_query(&mysql, "set names gbk"); //���ñ����ʽ��SET NAMES GBKҲ�У�������cmd����������
//														  //����0 ��ѯ�ɹ�������1��ѯʧ��
//	if (mysql_query(&mysql, query))        //ִ��SQL���
//	{
//		printf("Query failed (%s)\n", mysql_error(&mysql));
//		return false;
//	}
//	else
//	{
//		printf("query success\n");
//	}
//	//��ȡ�����
//	if (!(res = mysql_store_result(&mysql)))    //���sql�������󷵻صĽ����
//	{
//		printf("Couldn't get result from %s\n", mysql_error(&mysql));
//		return false;
//	}
//
//	//��ӡ��������
//	printf("number of dataline returned: %d\n", mysql_affected_rows(&mysql));
//
//	//��ȡ�ֶε���Ϣ
//	char *str_field[32];  //����һ���ַ�������洢�ֶ���Ϣ
//	for (int i = 0; i<4; i++)   //����֪�ֶ�����������»�ȡ�ֶ���
//	{
//		str_field[i] = mysql_fetch_field(res)->name;
//	}
//	for (int i = 0; i<4; i++)   //��ӡ�ֶ�
//		printf("%10s\t", str_field[i]);
//	printf("\n");
//	//��ӡ��ȡ������
//	while (column = mysql_fetch_row(res))   //����֪�ֶ���������£���ȡ����ӡ��һ��
//	{
//		printf("%10s\t%10s\t%10s\t%10s\n", column[0], column[1], column[2], column[3]);  //column��������
//	}
//	return true;
//}
//
//bool QueryDatabase3()
//{
//	//mysql_query(&mysql, "set names gbk");
//	//����0 ��ѯ�ɹ�������1��ѯʧ��
//	if (mysql_query(&mysql, "select * from data where Id=1"))        //ִ��SQL���
//	{
//		//printf("Query failed (%s)\n", mysql_error(&mysql));
//		return false;
//	}
//	res = mysql_store_result(&mysql);
//	//��ӡ��������
//	//printf("number of dataline returned: %d\n", mysql_affected_rows(&mysql));
//	for (int i = 0; fd = mysql_fetch_field(res); i++)  //��ȡ�ֶ���
//		strcpy(field[i], fd->name);
//	int j = mysql_num_fields(res);  // ��ȡ����
//									//for (int i = 0; i<j; i++)  //��ӡ�ֶ�
//									//	printf("%10s\t", field[i]);
//									//printf("\n");
//	while (column = mysql_fetch_row(res))
//	{
//		int temp_sail, temp_rudder, temp_xPosition, temp_yPosition, temp_angle;
//		temp_rudder = atof(column[2]);
//		temp_sail = atof(column[3]);
//		temp_xPosition = atof(column[4]);
//		temp_yPosition = atof(column[5]);
//		temp_angle = atof(column[6]);
//		//servo1 = temp_sail;
//		//servo2 = temp_rudder;
//		//for (int i = 0; i < j; i++)
//		//	printf("%10s\t", column[i]);
//		//printf("\n");
//	}
//	return true;
//}
