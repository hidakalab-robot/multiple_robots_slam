#include <ros/ros.h>
#include <class_test/basic_process.h>



class DuplicateProcess
{
public:
	/*コンストラクタ*/
	DuplicateProcess();
	/*デストラクタ*/
	~DuplicateProcess(){};
	bool duplicate_detection(void);
};
