/*
 *  AbstractFeatureMatcher.h
 *  SfMToyExample
 *
 *  Created by Roy Shilkrot on 8/6/12.
 *  Copyright 2012 MIT. All rights reserved.
 *
 */
#pragma once

#include "IFeatureMatcher.h"

class AbstractFeatureMatcher : public IFeatureMatcher {
protected:
	bool use_gpu;
public:
	AbstractFeatureMatcher(bool _use_gpu):use_gpu(_use_gpu) {}
};