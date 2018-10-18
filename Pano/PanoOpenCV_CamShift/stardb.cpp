#pragma warning( disable : 4996) 
//-----------MySQL
//#include "position.h"
#include <stdio.h>
#include <WinSock.h>  //一定要包含这个，或者winsock2.h
#include "include/mysql.h"    //引入mysql头文件(一种方式是在vc目录里面设置，一种是文件夹拷到工程目录，然后这样包含)
#include <Windows.h>
//包含附加依赖项，也可以在工程--属性里面设置
#pragma comment(lib,"wsock32.lib")
#pragma comment(lib,"libmysql.lib")
MYSQL mysql; //mysql连接
MYSQL_FIELD *fd;  //字段列数组
char field[32][32];  //存字段名二维数组
MYSQL_RES *res; //这个结构代表返回行的一个查询结果集
MYSQL_ROW column; //一个行数据的类型安全(type-safe)的表示，表示数据行的列
char query[150]; //查询语句

bool ConnectDatabase()
{
	//初始化mysql
	mysql_init(&mysql);  //连接mysql，数据库
	
						 //返回false则连接失败，返回true则连接成功
	if (!(mysql_real_connect(&mysql, "192.168.0.104", "root", "root", "STAr", 0, NULL, 0))) //中间分别是主机，用户名，密码，数据库名，端口号（可以写默认0或者3306等），可以先写成参数再传进去
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
	//释放资源
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
//	sprintf(query, "select rudder from data where id=1"); //执行查询语句，这里是查询所有，user是表名，不用加引号，用strcpy也可以
//														  //mysql_query(&mysql, "set names gbk"); //设置编码格式（SET NAMES GBK也行），否则cmd下中文乱码
//														  //返回0 查询成功，返回1查询失败
//	if (mysql_query(&mysql, query))        //执行SQL语句
//	{
//		printf("Query failed (%s)\n", mysql_error(&mysql));
//		return false;
//	}
//	else
//	{
//		printf("query success\n");
//	}
//	//获取结果集
//	if (!(res = mysql_store_result(&mysql)))    //获得sql语句结束后返回的结果集
//	{
//		printf("Couldn't get result from %s\n", mysql_error(&mysql));
//		return false;
//	}
//
//	//打印数据行数
//	printf("number of dataline returned: %d\n", mysql_affected_rows(&mysql));
//
//	//获取字段的信息
//	char *str_field[32];  //定义一个字符串数组存储字段信息
//	for (int i = 0; i<4; i++)   //在已知字段数量的情况下获取字段名
//	{
//		str_field[i] = mysql_fetch_field(res)->name;
//	}
//	for (int i = 0; i<4; i++)   //打印字段
//		printf("%10s\t", str_field[i]);
//	printf("\n");
//	//打印获取的数据
//	while (column = mysql_fetch_row(res))   //在已知字段数量情况下，获取并打印下一行
//	{
//		printf("%10s\t%10s\t%10s\t%10s\n", column[0], column[1], column[2], column[3]);  //column是列数组
//	}
//	return true;
//}
//
//bool QueryDatabase3()
//{
//	//mysql_query(&mysql, "set names gbk");
//	//返回0 查询成功，返回1查询失败
//	if (mysql_query(&mysql, "select * from data where Id=1"))        //执行SQL语句
//	{
//		//printf("Query failed (%s)\n", mysql_error(&mysql));
//		return false;
//	}
//	res = mysql_store_result(&mysql);
//	//打印数据行数
//	//printf("number of dataline returned: %d\n", mysql_affected_rows(&mysql));
//	for (int i = 0; fd = mysql_fetch_field(res); i++)  //获取字段名
//		strcpy(field[i], fd->name);
//	int j = mysql_num_fields(res);  // 获取列数
//									//for (int i = 0; i<j; i++)  //打印字段
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
