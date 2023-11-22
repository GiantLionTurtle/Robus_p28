
#include "Camera.hpp"

#include <Constants.hpp>
#include <Utils/Geometry.hpp>

#define CLAW_SIGNATURE 5
#define RED_SIGNATURE 1
#define GREEN_SIGNATURE 4
#define BLUE_SIGNATURE 3
#define YELLOW_SIGNATURE 2


namespace p28 {

static mt::i32Vec2 claw_pos(55, 195);
static mt::i32Box claw_box{.bottomLeft=mt::i32Vec2(20, 180), .topRight=mt::i32Vec2(70, 210)};

void Camera::init()
{
	pixy.init();
	pixy.setLamp(1, 1);
	pixy.setCameraBrightness(70);
}

mt::i32Vec2 centroid(Block block)
{
	return mt::i32Vec2(block.m_x + block.m_width/2, block.m_y + block.m_height/2);
}
Pair<mt::i32Vec2, bool> Camera::blockOffset(int color)
{
	// grab blocks!
	pixy.ccc.getBlocks();

	mt::i32Vec2 target_centroid(0);

	int biggest_block_size = 0;
	// Serial.print("n blocks: ");
	// Serial.println(pixy.ccc.numBlocks);
	for(int i = 0; i < pixy.ccc.numBlocks; ++i) {
		auto block = pixy.ccc.blocks[i];
		// block.print();
		if(signature_to_color(block.m_signature) == color) {
			int block_size = mt::magnitude2(mt::i32Vec2(block.m_width, block.m_height));
			if(block_size < biggest_block_size)
				continue;
			target_centroid = centroid(block);
			biggest_block_size = block_size;
		}
	}
	// Serial.print("blocl centroid: ");
	// print(target_centroid);
	// Serial.println();

	// either no block of the correct color was found or the claw was not found
	if(target_centroid != mt::i32Vec2(0)) {
		// Serial.print("Target centroid: ");
		// println(target_centroid);
		bool inside = claw_box.point_inside(target_centroid);
		// if(inside) {
		// 	Serial.println("INSIDE");
		// }
		mt::i32Vec2 diff = claw_pos - target_centroid;
		return {{ mt::clamp(diff.x, -100, 100), mt::clamp(diff.y, -100, 100) },  inside };
	}
	return { { 0 }, false };
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