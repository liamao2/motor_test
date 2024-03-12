

typedef struct
{ 
struct 
{ 
unsigned short ch0; 
unsigned short ch1; 
unsigned short ch2; 
unsigned short ch3; 
unsigned char  s1; 
unsigned char  s2; 
}rc; 
struct 
{ 
unsigned short x; 
unsigned short y; 
unsigned short z; 
unsigned short press_l; 
unsigned short press_r; 
}mouse; 
struct 
{ 
unsigned short v; 
}key; 
}Rc_ctrl; 



