
#include "Camera.hpp"

#include <Constants.hpp>
#include <Utils/Geometry.hpp>

#define RED_SIGNATURE 1
#define GREEN_SIGNATURE 2
#define BLUE_SIGNATURE 3
#define YELLOW_SIGNATURE 4

namespace p28 {

#ifdef ENABLE_CAMERA

void Camera::init()
{
	pixy.init();
	// pixy.setLamp(1, 1);
	pixy.setCameraBrightness(Tracking::kCameraBrightness);
}

mt::i32Vec2 centroid(Block block)
{
	return mt::i32Vec2(block.m_x + block.m_width/2, block.m_y + block.m_height/2);
}
mt::i32Box box(Block block)
{
	return mt::i32Box{.bottomLeft=mt::i32Vec2(block.m_x, block.m_y), .topRight=mt::i32Vec2(block.m_x+block.m_width, block.m_y+block.m_height)};
}
void Camera::blockOffset(int targcolor, mt::Vec2& offset, bool& in_claw, int& color)
{
	using namespace Tracking;

	// grab blocks!
	pixy.ccc.getBlocks();
	// if(pixy.ccc.getBlocks(false) < 0) {
	// 	return;
	// }
	// bool error = true;
	// for(int i = 0; i < 5; ++i) {
	// 	if(pixy.ccc.getBlocks(false, 255, 12) >= 0) {
	// 		error = false;
	// 		break;
	// 	}
	// 	delayMicroseconds(200);
	// }
	// if(error)
	// 	return;

	color = targcolor;
	int target_ind = -1;
	int32_t biggest_block_size = 0;
	for(int i = 0; i < pixy.ccc.numBlocks; ++i) {
		auto block = pixy.ccc.blocks[i];
		int block_color = signature_to_color(block.m_signature);
		// Serial.print("Block ");
		// Serial.println(block_color);
		if(block_color == -1 || (block_color != targcolor && targcolor != kAllColors))
			continue;

		// Block size with advantage for closer blocks
		int32_t block_size = mt::magnitude2(mt::i32Vec2(block.m_width, block.m_height))/4 * 
							(kCamViewport.y-block.m_y)/10;// * 
							// (kCamViewport.x-block.m_x)/10 * 
							// (block.m_age/10+1);
		// Serial.print("Blocksize ");
		// mt::print(mt::i32Vec2(block.m_width, block.m_height));
		// Serial.print(block.m_x);
		// Serial.print(",  ");
		// Serial.println(block.m_y);
		// Serial.print(",  ");
		// Serial.println(block_size);
		if(block_size < biggest_block_size || block.m_y < 70 || block.m_x < 15 || block.m_x > 270)
			continue;
		target_ind = i;
		biggest_block_size = block_size;
		if(targcolor == kAllColors) {
			color = block_color;
		}
	}

	// either no block of the correct color was found or the claw was not found
	if(target_ind != -1) {
		mt::i32Vec2 target_centroid = centroid(pixy.ccc.blocks[target_ind]);
		mt::i32Box target_box = box(pixy.ccc.blocks[target_ind]);
		
		in_claw = kClawBox.box_inside(target_box);	
		float diff_x = kClawLine.dist_signed(mt::Vec2(target_centroid.x, target_centroid.y));
		offset = { 	mt::clamp(diff_x, -kClampOffset, kClampOffset), 
					mt::clamp(kClawPos.y-target_centroid.y, -kClampOffset, kClampOffset) };
	} else {
		offset = { 0.0 };
		in_claw = false;
		color = -1;
	}
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