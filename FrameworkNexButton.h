#ifndef __FRAMEWORK_NEXBUTTON_H__
#define __FRAMEWORK_NEXBUTTON_H__

#include "NexButton.h"

class FrameworkNexButton : public NexButton 
{

public: 

    	FrameworkNexButton(uint8_t pid, uint8_t cid, const char *name);  
	bool setPic(uint32_t number);
	bool setBackgroundPic(uint32_t pic);
    	bool enableTouch(uint32_t pic = 0, uint32_t pic2 = 0);
    	bool disableTouch(uint32_t pic = 0); 
	bool refresh(); 
};

#endif /* #ifndef __FRAMEWORK_NEXBUTTON_H__ */
