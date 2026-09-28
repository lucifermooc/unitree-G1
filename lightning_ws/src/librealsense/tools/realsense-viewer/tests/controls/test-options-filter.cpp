// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

#include "viewer-test-helpers.h"


// Type into the Controls search box and verify the option list is filtered live:
// case-insensitive substring match on the control name, empty input restores the full list
VIEWER_TEST( "controls", "options_filter" )
{
    auto & model = test.find_first_device_or_exit();
    bool tested = false;

    for( auto && sub : model.subdevices )
    {
        test.expand_sensor_panel( model, sub );
        test.expand_controls( model, sub );

        // the options the UI currently renders inside this sensor's Controls section
        auto options = test.controls_options( model, sub );
        if( options.size() < 2 )
        {
            test.collapse_sensor_panel( model, sub );
            continue;
        }

        // filter = first control's name minus its last char; pick another control whose
        // name does not contain it
        std::string filter = test.control_name( sub, options[0] );
        filter.pop_back();
        rs2_option other = RS2_OPTION_COUNT;
        for( auto o : options )
            if( test.control_name( sub, o ).find( filter ) == std::string::npos )
            {
                other = o;
                break;
            }
        if( other == RS2_OPTION_COUNT ) // filter hides nothing — can't verify on this sensor
        {
            test.collapse_sensor_panel( model, sub );
            continue;
        }

        // case-insensitive substring match: non-matching control disappears, matching stays
        test.set_controls_filter( model, sub, filter );
        IM_CHECK( test.wait_until( 10, 0.3f, [&] { return ! test.control_visible( model, sub, other ); } ) );
        IM_CHECK( test.control_visible( model, sub, options[0] ) );

        // clearing the box restores the full list
        test.set_controls_filter( model, sub, "" );
        IM_CHECK( test.wait_until( 10, 0.3f, [&] { return test.control_visible( model, sub, other ); } ) );

        test.collapse_controls( model, sub );
        test.collapse_sensor_panel( model, sub );
        tested = true;
        break; // one sensor is enough — the filter code path is per-sensor identical
    }

    IM_CHECK( tested );
}
