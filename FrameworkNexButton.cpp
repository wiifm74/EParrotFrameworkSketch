#include "FrameworkNexButton.h"

FrameworkNexButton::FrameworkNexButton(uint8_t pid, uint8_t cid, const char *name)
    :NexButton(pid, cid, name)
{
}

bool FrameworkNexButton::setPic(uint32_t pic)
{
    char buf[10] = {0};
    String cmd;
    
    utoa(pic, buf, 10);
    cmd += getObjName();
    cmd += ".pic=";
    cmd += buf;
    sendCommand(cmd.c_str());
	refresh();
    return recvRetCommandFinished();
}

bool FrameworkNexButton::setBackgroundPic(uint32_t pic)
{
    char buf[10] = {0};
    String cmd;
    
    utoa(pic, buf, 10);
    cmd += getObjName();
    cmd += ".pic2=";
    cmd += buf;
    sendCommand(cmd.c_str());
	refresh();
    return recvRetCommandFinished();
}

bool FrameworkNexButton::enableTouch(uint32_t pic = 0, uint32_t pic2 = 0)
{
	String cmd;
	cmd+= "tsw ";
	cmd += getObjName();
    	cmd += ", 1";
	sendCommand(cmd.c_str());
	if (pic != 0) {
		setPic(pic);
	}
	if (pic2 != 0) {
		setBackgroundPic(pic);
	}
	return recvRetCommandFinished();
}

bool FrameworkNexButton::disableTouch(uint32_t pic = 0)
{
	String cmd;
	cmd+= "tsw ";
	cmd += getObjName();
    	cmd += ", 0";
	sendCommand(cmd.c_str());
	if (pic != 0) {
		setPic(pic);
		setBackgroundPic(pic);
	}
	return recvRetCommandFinished();
}

bool FrameworkNexButton::refresh()
{
	String cmd;
	cmd+= "ref ";
	cmd += getObjName();
	sendCommand(cmd.c_str());
    	return recvRetCommandFinished();
}