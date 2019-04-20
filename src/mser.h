#ifndef MSER_H
#define MSER_H

#include <vector>
#include <stdint.h>

public:
	/// A Maximally Stable Extremal Region.
	struct Region
	{
		int level_; ///< Level at which the region is processed.
		int pixel_; ///< Index of the initial pixel (y * width + x).
		int area_; ///< Area of the region (moment zero).
		double moments_[5]; ///< First and second moments of the region (x, y, x^2, xy, y^2).
		double variation_; ///< MSER variation.
		
		Region(int level = 256, int pixel = 0);
		
	private:
		bool stable_; // Flag indicating if the region is stable
		Region * parent_; // Pointer to the parent region
		Region * child_; // Pointer to the first child
		Region * next_; // Pointer to the next (sister) region
		
		void accumulate(int x, int y);
		void merge(Region * child);
		void detect(int delta, int minArea, int maxArea, double maxVariation, double minDiversity,
					std::vector<Region> & regions);
		void process(int delta, int minArea, int maxArea, double maxVariation);
		bool check(double variation, int area) const;
		void save(double minDiversity, std::vector<Region> & regions);
		
		friend class MSER;
	};
	
	MSER(int delta = 2, double minArea = 0.0001, double maxArea = 0.5, double maxVariation = 0.5,
		 double minDiversity = 0.33, bool eight = false);
	
	void operator()(const uint8_t * bits, int width, int height, std::vector<Region> & regions);
	
private:
	void processStack(int newPixelGreyLevel, int pixel, std::vector<Region *> & regionStack);
	
	std::ptrdiff_t doublePool(std::vector<Region *> & regionStack);
	
	int delta_;
	double minArea_;
	double maxArea_;
	double maxVariation_;
	double minDiversity_;
	bool eight_;
	
	std::vector<Region> pool_;
	std::size_t poolIndex_;
};

#endif
