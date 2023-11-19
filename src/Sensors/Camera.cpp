
#include "Camera.hpp"

#include <Constants.hpp>

#define CLAW_SIGNATURE 5
#define RED_SIGNATURE 0
#define GREEN_SIGNATURE 1
#define BLUE_SIGNATURE 3
#define YELLOW_SIGNATURE 2


namespace p28 {

void Camera::init()
{
	pixy.init();
}

mt::i32Vec2 centroid(Block block)
{
	return mt::i32Vec2(block.m_x + block.m_width/2, block.m_y + block.m_height/2);
}
mt::i32Vec2 Camera::blockOffset(int color)
{
	// grab blocks!
	pixy.ccc.getBlocks();

	mt::i32Vec2 claw_centroid(0);
	mt::i32Vec2 target_centroid(0);

	int biggest_block_size = 0;
	for(int i = 0; i < pixy.ccc.numBlocks; ++i) {
		auto block = pixy.ccc.blocks[i];
		if(signature_to_color(block.m_signature) == color) {
			int block_size = mt::magnitude2(mt::i32Vec2(block.m_width, block.m_height));
			if(block_size < biggest_block_size)
				continue;
			target_centroid = centroid(block);
			biggest_block_size = block_size;
		}
		if(block.m_signature == CLAW_SIGNATURE) {
			claw_centroid = centroid(block);
		}
	}

	// either no block of the correct color was found or the claw was not found
	if(claw_centroid == mt::i32Vec2(0) || target_centroid == mt::i32Vec2(0)) {
		mt::i32Vec2 diff = claw_centroid - target_centroid;
		return { mt::clamp(diff.x, -40, 40), mt::clamp(diff.y, -40, 40) };
	}
	return { 0 };
}
int Camera::signature_to_color(int sig)
{
	switch(sig) {
	case RED_SIGNATURE: return kRed;
	case GREEN_SIGNATURE: return KGreen;
	case BLUE_SIGNATURE: return kBlue;
	case YELLOW_SIGNATURE: return kYellow;
	default: return -1;
	}
}

} // !p28