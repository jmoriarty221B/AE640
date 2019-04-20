#include "mser.h"

#include <algorithm>
#include <cassert>
#include <limits>

using namespace std;

MSER::Region::Region(int level, int pixel) : level_(level), pixel_(pixel), area_(0),
variation_(numeric_limits<double>::infinity()), stable_(false), parent_(0), child_(0), next_(0)
{
	fill_n(moments_, 5, 0.0);
}

inline void MSER::Region::accumulate(int x, int y)
{
	++area_;
	moments_[0] += x;
	moments_[1] += y;
	moments_[2] += x * x;
	moments_[3] += x * y;
	moments_[4] += y * y;
}

void MSER::Region::merge(Region * child)
{
	assert(!child->parent_);
	assert(!child->next_);
	
	area_ += child->area_;
	moments_[0] += child->moments_[0];
	moments_[1] += child->moments_[1];
	moments_[2] += child->moments_[2];
	moments_[3] += child->moments_[3];
	moments_[4] += child->moments_[4];
	
	child->next_ = child_;
	child_ = child;
	child->parent_ = this;
}

void MSER::Region::process(int delta, int minArea, int maxArea, double maxVariation)
{
	const Region * parent = this;
	
	while (parent->parent_ && (parent->parent_->level_ <= (level_ + delta)))
		parent = parent->parent_;
	
	variation_ = static_cast<double>(parent->area_ - area_) / area_;
	
	const bool stable = (!parent_ || (variation_ <= parent_->variation_)) &&
						(area_ >= minArea) && (area_ <= maxArea) && (variation_ <= maxVariation);
	
	for (Region * child = child_; child; child = child->next_) {
		child->process(delta, minArea, maxArea, maxVariation);
		
		if (stable && (variation_ < child->variation_))
			stable_ = true;
	}
	
	if (!child_ && stable)
		stable_ = true;
}

bool MSER::Region::check(double variation, int area) const
{
	if (area_ <= area)
		return true;
	
	if (stable_ && (variation_ < variation))
		return false;
	
	for (Region * child = child_; child; child = child->next_)
		if (!child->check(variation, area))
			return false;
	
	return true;
}

void MSER::Region::save(double minDiversity, vector<Region> & regions)
{
	if (stable_) {
		const int minParentArea = area_ / (1.0 - minDiversity) + 0.5;
		
		const Region * parent = this;
		
		while (parent->parent_ && (parent->parent_->area_ < minParentArea)) {
			parent = parent->parent_;
			
			if (parent->stable_ && (parent->variation_ <= variation_)) {
				stable_ = false;
				break;
			}
		}
		
		if (stable_) {
			const int maxChildArea = area_ * (1.0 - minDiversity) + 0.5;
			
			if (!check(variation_, maxChildArea))
				stable_ = false;
		}
		
		if (stable_) {
			regions.push_back(*this);
			regions.back().parent_ = 0;
			regions.back().child_ = 0;
			regions.back().next_ = 0;
		}
	}
	
	for (Region * child = child_; child; child = child->next_)
		child->save(minDiversity, regions);
}

void MSER::Region::detect(int delta, int minArea, int maxArea, double maxVariation,
						  double minDiversity, vector<Region> & regions)
{
	process(delta, minArea, maxArea, maxVariation);
	save(minDiversity, regions);
}

MSER::MSER(int delta, double minArea, double maxArea, double maxVariation, double minDiversity,
		   bool eight) : eight_(eight), delta_(delta), minArea_(minArea), maxArea_(maxArea),
maxVariation_(maxVariation), minDiversity_(minDiversity), pool_(256), poolIndex_(0)
{
	assert(delta > 0);
	assert(minArea >= 0.0);
	assert(maxArea <= 1.0);
	assert(minArea < maxArea);
	assert(maxVariation > 0.0);
	assert(minDiversity >= 0.0);
	assert(minDiversity < 1.0);
}

void MSER::operator()(const uint8_t * bits, int width, int height, vector<Region> & regions)
{
	vector<bool> accessible(width * height);
	vector<int> boundaryPixels[256];
	int priority = 256;
	vector<Region *> regionStack;
	regionStack.push_back(new (&pool_[poolIndex_++]) Region);
	
	int curPixel = 0;
	int curEdge = 0;
	int curLevel = bits[0];
	accessible[0] = true;
	
step_3:
	regionStack.push_back(new (&pool_[poolIndex_++]) Region(curLevel, curPixel));
	
	if (poolIndex_ == pool_.size())
		doublePool(regionStack);
	
	for (;;) {
		const int x = curPixel % width;
		const int y = curPixel / width;
		
		for (; curEdge < (eight_ ? 8 : 4); ++curEdge) {
			int neighborPixel = curPixel;
			
			if (eight_) {
				switch (curEdge) {
					case 0: if (x < width - 1) neighborPixel = curPixel + 1; break;
					case 1: if ((x < width - 1) && (y > 0)) neighborPixel = curPixel - width + 1; break;
					case 2: if (y > 0) neighborPixel = curPixel - width; break;
					case 3: if ((x > 0) && (y > 0)) neighborPixel = curPixel - width - 1; break;
					case 4: if (x > 0) neighborPixel = curPixel - 1; break;
					case 5: if ((x > 0) && (y < height - 1)) neighborPixel = curPixel + width - 1; break;
					case 6: if (y < height - 1) neighborPixel = curPixel + width; break;
					default: if ((x < width - 1) && (y < height - 1)) neighborPixel = curPixel + width + 1; break;
				}
			}
			else {
				switch (curEdge) {
					case 0: if (x < width - 1) neighborPixel = curPixel + 1; break;
					case 1: if (y < height - 1) neighborPixel = curPixel + width; break;
					case 2: if (x > 0) neighborPixel = curPixel - 1; break;
					default: if (y > 0) neighborPixel = curPixel - width; break;
				}
			}
			
			if (neighborPixel != curPixel && !accessible[neighborPixel]) {
				const int neighborLevel = bits[neighborPixel];
				accessible[neighborPixel] = true;
				
				if (neighborLevel >= curLevel) {
					boundaryPixels[neighborLevel].push_back(neighborPixel << 4);
					
					if (neighborLevel < priority)
						priority = neighborLevel;
				}
				else {
					boundaryPixels[curLevel].push_back((curPixel << 4) | (curEdge + 1));
					
					if (curLevel < priority)
						priority = curLevel;
					
					curPixel = neighborPixel;
					curEdge = 0;
					curLevel = neighborLevel;
					
					goto step_3;
				}
			}
		}
		
		regionStack.back()->accumulate(x, y);
		
		if (priority == 256) {
			regionStack.back()->detect(delta_, minArea_ * width * height,
									   maxArea_ * width * height, maxVariation_, minDiversity_,
									   regions);
			poolIndex_ = 0;
			return;
		}
		
		curPixel = boundaryPixels[priority].back() >> 4;
		curEdge = boundaryPixels[priority].back() & 15;
		
		boundaryPixels[priority].pop_back();
		
		while (boundaryPixels[priority].empty() && (priority < 256))
			++priority;
		
		const int newPixelGreyLevel = bits[curPixel];
		
		if (newPixelGreyLevel != curLevel) {
			curLevel = newPixelGreyLevel;
			
			processStack(newPixelGreyLevel, curPixel, regionStack);
		}
	}
}

void MSER::processStack(int newPixelGreyLevel, int pixel, vector<Region *> & regionStack)
{
	do {
		Region * top = regionStack.back();
		
		regionStack.pop_back();
		
		if (newPixelGreyLevel < regionStack.back()->level_) {
			regionStack.push_back(new (&pool_[poolIndex_++]) Region(newPixelGreyLevel, pixel));
			
			if (poolIndex_ == pool_.size())
				top = reinterpret_cast<Region *>(reinterpret_cast<char *>(top) +
												 doublePool(regionStack));
			
			regionStack.back()->merge(top);
			
			return;
		}
		
		regionStack.back()->merge(top);
	}
	while (newPixelGreyLevel > regionStack.back()->level_);
}

ptrdiff_t MSER::doublePool(vector<Region *> & regionStack)
{
	assert(!pool_.empty()); 
	
	vector<Region> newPool(pool_.size() * 2);
	copy(pool_.begin(), pool_.end(), newPool.begin());
	
	const ptrdiff_t offset = reinterpret_cast<char *>(&newPool[0]) -
								  reinterpret_cast<char *>(&pool_[0]);
	
	for (size_t i = 0; i < pool_.size(); ++i) {
		if (newPool[i].parent_)
			newPool[i].parent_ =
				reinterpret_cast<Region *>(reinterpret_cast<char *>(newPool[i].parent_) + offset);
		
		if (newPool[i].child_)
			newPool[i].child_ =
				reinterpret_cast<Region *>(reinterpret_cast<char *>(newPool[i].child_) + offset);
		
		if (newPool[i].next_)
			newPool[i].next_ =
				reinterpret_cast<Region *>(reinterpret_cast<char *>(newPool[i].next_) + offset);
	}
	
	for (size_t i = 0; i < regionStack.size(); ++i)
		regionStack[i] =
			reinterpret_cast<Region *>(reinterpret_cast<char *>(regionStack[i]) + offset);
	
	pool_.swap(newPool);
	
	return offset;
}
