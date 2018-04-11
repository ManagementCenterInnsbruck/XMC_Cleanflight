/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"


static uint32_t activeFeaturesLatch = 0;

void intFeatureSet(uint32_t mask, uint32_t *features)
{
    *features |= mask;
}

void intFeatureClear(uint32_t mask, uint32_t *features)
{
    *features &= ~(mask);
}

void intFeatureClearAll(uint32_t *features)
{
    *features = 0;
}

void latchActiveFeatures()
{
    activeFeaturesLatch = featureConfig()->enabledFeatures;
}

bool featureConfigured(uint32_t mask)
{
    return featureConfig()->enabledFeatures & mask;
}

bool feature(uint32_t mask)
{
    return activeFeaturesLatch & mask;
}

void featureSet(uint32_t mask)
{
    intFeatureSet(mask, &featureConfigMutable()->enabledFeatures);
}

void featureClear(uint32_t mask)
{
    intFeatureClear(mask, &featureConfigMutable()->enabledFeatures);
}

void featureClearAll()
{
    intFeatureClearAll(&featureConfigMutable()->enabledFeatures);
}

uint32_t featureMask(void)
{
    return featureConfig()->enabledFeatures;
}
