#ifndef P28_LINEDETECTOR_HPP_
#define P28_LINEDETECTOR_HPP_

namespace p28{
	char get_ir_line();
	bool get_ir(int index);
	bool is_active(char line, short index);
	int total(char line);
} // !p28
#endif