//
// Created by abuchegger on 28.05.18.
//
#include <gtest/gtest.h>
#include <arti_async_utils/async_resource_update.h>
#include <arti_async_utils/buffered_resource.h>
#include <arti_async_utils/event_pipeline_stage.h>
#include <arti_async_utils/interrupt_exception.h>
#include <arti_async_utils/lockable_resource.h>
#include <arti_async_utils/periodic_pipeline_stage.h>
#include <arti_async_utils/pipeline.h>
#include <arti_async_utils/pipeline_stage.h>
#include <arti_async_utils/pipeline_stage_base.h>

// Currently, this test is just a syntax check for the include files.


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
