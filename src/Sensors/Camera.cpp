
#include "Camera.hpp"

#include <Constants.hpp>
#include <Utils/Geometry.hpp>

#define CLAW_SIGNATURE 5
#define RED_SIGNATURE 1
#define GREEN_SIGNATURE 4
#define BLUE_SIGNATURE 3
#define YELLOW_SIGNATURE 2


namespace p28 {
	
static mt::Vec2 claw_pos(54, 167);
static mt::Line claw_line { .origin=claw_pos, .dir=mt::Vec2(86, 17)-claw_pos};
// static mt::i32Box claw_box{.bottomLeft=mt::i32Vec2(42, 178), .topRight=mt::i32Vec2(68, 200)};
static mt::i32Box claw_box{.bottomLeft=mt::i32Vec2(35, 165), .topRight=mt::i32Vec2(75, 210)};

#ifdef ENABLE_CAMERA

void Camera::init()
{
	pixy.init();
	pixy.setLamp(1, 1);
	pixy.setCameraBrightness(138);
}

mt::i32Vec2 centroid(Block block)
{
	return mt::i32Vec2(block.m_x + block.m_width/2, block.m_y + block.m_height/2);
}
mt::i32Box box(Block block)
{
	return mt::i32Box{.bottomLeft=mt::i32Vec2(block.m_x, block.m_y), .topRight=mt::i32Vec2(block.m_x+block.m_width, block.m_y+block.m_height)};
}
Pair<mt::Vec2, bool> Camera::blockOffset(int color)
{
	// grab blocks!
	pixy.ccc.getBlocks();

	int target_ind = -1;

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
			target_ind = i;
			biggest_block_size = block_size;
		}
	}
	// Serial.print("blocl ind: ");
	// Serial.print(target_ind);
	// Serial.println();

	// either no block of the correct color was found or the claw was not found
	if(target_ind != -1) {
		mt::i32Vec2 target_centroid = centroid(pixy.ccc.blocks[target_ind]);
		mt::i32Box target_box = box(pixy.ccc.blocks[target_ind]);
		
		// Serial.print("Target centroid: ");
		// println(target_centroid);
		bool inside = claw_box.box_inside(target_box);


		// if(inside) {
		// 	Serial.println("INSIDE");
		// }
	
		float diff_x = claw_line.dist_signed(mt::Vec2(target_centroid.x, target_centroid.y));
		// Serial.print("Diff x ");
		// Serial.println(diff_x);
		return {{ mt::clamp(diff_x, -50, 50), mt::clamp(claw_pos.y-target_centroid.y, -50, 50) },  inside };
	}
	return { { 0 }, false };
}
#else 
void Camera::init()
{

}

Pair<mt::i32Vec2, bool> Camera::blockOffset(int color)
{
	return { { 0 }, false };
}
#endif
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