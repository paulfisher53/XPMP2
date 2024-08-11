/// @file       2D.cpp
/// @brief      Implementation of 2-D routines, like drawing aircraft labels
/// @details    2-D drawing is a bit "unnatural" as the aircraft are put into a 3-D world.
///             These functions require to turn 3D coordinates into 2D coordinates.
/// @see        Laminar's sample code at https://developer.x-plane.com/code-sample/coachmarks/
///             is the basis, then been taken apart.
/// @author     Laminar Research
/// @author     Birger Hoppe
/// @copyright  (c) 2020 Birger Hoppe
/// @copyright  Permission is hereby granted, free of charge, to any person obtaining a
///             copy of this software and associated documentation files (the "Software"),
///             to deal in the Software without restriction, including without limitation
///             the rights to use, copy, modify, merge, publish, distribute, sublicense,
///             and/or sell copies of the Software, and to permit persons to whom the
///             Software is furnished to do so, subject to the following conditions:\n
///             The above copyright notice and this permission notice shall be included in
///             all copies or substantial portions of the Software.\n
///             THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
///             IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
///             FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
///             AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
///             LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
///             OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
///             THE SOFTWARE.

#include "XPMP2.h"

#define DEBUG_ENABLE_AC_LABELS  "Aircraft labels %s"

namespace XPMP2 {

//
// MARK: 2-D projection calculations
//

// Data refs we need
static XPLMDataRef drMatrixWrld     = nullptr;  ///< sim/graphics/view/world_matrix
static XPLMDataRef drMatrixProj     = nullptr;  ///< sim/graphics/view/projection_matrix_3d
static XPLMDataRef drScreenWidth    = nullptr;  ///< sim/graphics/view/window_width
static XPLMDataRef drScreenHeight   = nullptr;  ///< sim/graphics/view/window_height
static XPLMDataRef drVisibility     = nullptr;  ///< sim/graphics/view/visibility_effective_m or sim/weather/visibility_effective_m
static XPLMDataRef drFieldOfView    = nullptr;  ///< sim/graphics/view/field_of_view_deg

/// world matrix (updates once per cycle)
static float gMatrixWrld[16];
/// projection matrix (updated once per cycle)
static float gMatrixProj[16];
/// Screen size (with, height)
static float gScreenW, gScreenH;
/// Field of view
static float gFOV;

/// 4x4 matrix transform of an XYZW coordinate - this matches OpenGL matrix conventions.
static void mult_matrix_vec(float dst[4], const float m[16], const float v[4])
{
    dst[0] = v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + v[3] * m[12];
    dst[1] = v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + v[3] * m[13];
    dst[2] = v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + v[3] * m[14];
    dst[3] = v[0] * m[3] + v[1] * m[7] + v[2] * m[11] + v[3] * m[15];
}


/// Once per cycle read necessary matrices from X-Plane
static void read_matrices ()
{
    // Read the model view and projection matrices from this frame
    XPLMGetDatavf(drMatrixWrld,gMatrixWrld,0,16);
    XPLMGetDatavf(drMatrixProj,gMatrixProj,0,16);
    
    // Read the screen size (won't change often if at all...but could!)
    gScreenW = (float)XPLMGetDatai(drScreenWidth);
    gScreenH = (float)XPLMGetDatai(drScreenHeight);
    
    // Field of view
    gFOV = XPLMGetDataf(drFieldOfView);
}

// This drawing callback will draw a label to the screen where the

/// @brief Converts 3D local coordinates to 2D screen coordinates
/// @note Requires matrices to be set up already by a call to read_matrices()
/// @return Are coordinates visible? (Otherwise they are "in the back" of the camera)
static bool ConvertTo2d(const float x, const float y, const float z,
                        int& out_x, int& out_y)
{
    // the position to convert
    const float afPos[4] = { x, y, z, 1.0f };
    float afEye[4], afNdc[4];
    
    // Simulate the OpenGL transformation to get screen coordinates.
    mult_matrix_vec(afEye, gMatrixWrld, afPos);
    mult_matrix_vec(afNdc, gMatrixProj, afEye);
    
    afNdc[3] = 1.0f / afNdc[3];
    afNdc[0] *= afNdc[3];
    afNdc[1] *= afNdc[3];
    afNdc[2] *= afNdc[3];
    
    out_x = (int)std::lround(gScreenW * (afNdc[0] * 0.5f + 0.5f));
    out_y = (int)std::lround(gScreenH * (afNdc[1] * 0.5f + 0.5f));
    
    // afNdc[2] is basically the Z value
    if (glob.UsingModernGraphicsDriver())
        // Vulkan z-axis NDC is [0,1]
        return 0.0f <= afNdc[2] && afNdc[2] <= 1.0f;
    else
        // OGL z-axis is [-1,1]
        return -1.0f <= afNdc[2] && afNdc[2] <= 1.0;
}

void DrawTranslucentBox(float left, float top, float right, float bottom, float r, float g, float b, float a)
{
    XPLMSetGraphicsState(
            0,  // inEnableFog
            0,  // inNumberTexUnits
            0,  // inEnableLighting
            0,  // inEnableAlphaTesting
            1,  // inEnableAlphaBlending
            1,  // inEnableDepthTesting
            0   // inEnableDepthWriting
        );

   // Set the color (r, g, b, a) where 'a' is the alpha (transparency)
   glColor4f(r, g, b, a);

   // Draw the rectangle
    glBegin(GL_QUADS);
    {
        glVertex2f(left, bottom);
        glVertex2f(left, top);
        glVertex2f(right, top);
        glVertex2f(right, bottom);
    }
   glEnd();

}

//
// MARK: Drawing Control
//

/// @brief Write the labels of all aircraft
/// @see This code bases on the last part of `XPMPDefaultPlaneRenderer` of the original libxplanemp
/// @author Ben Supnik, Chris Serio, Chris Collins, Birger Hoppe
void TwoDDrawLabels ()
{
    XPLMCameraPosition_t posCamera;
    
    // short-cut if label-writing is completely switched off
    if (!glob.bDrawLabels || glob.eLabelOverride == SWITCH_CFG_OFF) return;
    
    // Set up required matrices once
    read_matrices();
    
    // Determine the maximum distance for label drawing.
    // Depends on current actual visibility as well as a configurable maximum
    XPLMReadCameraPosition(&posCamera);
    const float maxLabelDist = (std::min(glob.maxLabelDist,
                                         (glob.bLabelCutOffAtVisibility && drVisibility) ? XPLMGetDataf(drVisibility) : glob.maxLabelDist)
                                * posCamera.zoom);    // Labels get easier to see when users zooms.
    
    // Loop over all aircraft and draw their labels
    for (auto& p: glob.mapAc)
    {
        Aircraft& ac = *p.second;
        try {
            // skip if a/c is not rendered or label not to be drawn
            if (!ac.IsRendered() ||
                !(ac.ShallDrawLabel() || glob.eLabelOverride == SWITCH_CFG_ON))
                continue;
        
            // Exit if aircraft is father away from camera than we would draw labels for
            if (ac.GetCameraDist() > maxLabelDist)
                continue;
            
            // Vertical label offset: Idea is to place the label _above_ the plane
            // (as opposed to across), but finding the exact height of the plane
            // would require scanning the .obj file (well...we do so in CSLObj::FetchVertOfsFromObjFile (), but don't want to scan _every_ file)
            // We just use 3 fixed offset depending on the wake-turbulence category
            float vertLabelOfs = 10.0f;
            const XPMP2::CSLModel* pCSLMdl = ac.GetModel();
            if (pCSLMdl) {              // there's no reason why there shouldn't be a CSL model...just to be safe, though
                switch (pCSLMdl->GetDoc8643().wtc[0])
                {
                    case 'L': vertLabelOfs = 6.0f; break;
                    case 'H': vertLabelOfs = 11.0f; break;
                }
            }
        
            // Map the 3D coordinates of the aircraft to 2D coordinates of the flat screen
            int x = -1, y = -1;
            if (!ConvertTo2d(ac.drawInfo.x,
                             ac.drawInfo.y + vertLabelOfs,  // make the label appear above the plane
                             ac.drawInfo.z, x, y))
                continue;                           // label not visible

            // Finally: Draw the label
            int labelWidth = XPLMMeasureString(xplmFont_Proportional, ac.label.c_str(), ac.label.size());
            int subLabelWidth = XPLMMeasureString(xplmFont_Proportional, ac.subLabel.c_str(), ac.subLabel.size());
            int boxWidth = labelWidth;
            if (subLabelWidth > boxWidth) {
                boxWidth = subLabelWidth;
            }

            // Center the box by adjusting the x-coordinate
            int boxXStart = x - (boxWidth / 2);

            // Draw the translucent box
            DrawTranslucentBox(boxXStart - 5, y + 15, boxXStart + boxWidth + 5, y - 10, ac.colBackground[0], ac.colBackground[1], ac.colBackground[2], ac.colBackground[3]);

            // Draw the main label centered within the box
            int labelX = boxXStart + (boxWidth - labelWidth) / 2;
            XPLMDrawString(ac.colLabel, labelX, y, (char*)ac.label.c_str(), NULL, xplmFont_Proportional);

            // Draw the sub-label if it exists, also centered within the box
            if (!ac.subLabel.empty()) {
                float gray[4] = {1.0f, 1.0f, 1.0f, 0.6f};
                int subLabelX = boxXStart + (boxWidth - subLabelWidth) / 2;
                XPLMDrawString(gray, subLabelX, y - 25, (char*)ac.subLabel.c_str(), NULL, xplmFont_Proportional);
            }
        }
        CATCH_AC(ac)
    }
}

/// Drawing callback, called by X-Plane in every drawing cycle
int CPLabelDrawing (XPLMDrawingPhase     /*inPhase*/,
                    int                  /*inIsBefore*/,
                    void *               /*inRefcon*/)
{
    TwoDDrawLabels();
    return 1;
}


/// Activate actual label drawing, esp. set up drawing callback
void TwoDActivate ()
{
    // Register the actual drawing func.
    // This actually is a deprecated call, but it is at the same time the recommended way to draw labels,
    // see https://developer.x-plane.com/code-sample/coachmarks/
    XPLMRegisterDrawCallback(CPLabelDrawing,
                             xplm_Phase_Window,
                             1,                        // after
                             nullptr);
}


/// Deactivate actual label drawing, esp. stop drawing callback
void TwoDDeactivate ()
{
    // Unregister the drawing callback
    // This actually is a deprecated call, but it is at the same time the recommended way to draw labels,
    // see https://developer.x-plane.com/code-sample/coachmarks/
    XPLMUnregisterDrawCallback(CPLabelDrawing, xplm_Phase_Window, 1, nullptr);
}


// Initialize the module
void TwoDInit ()
{
    // initialize dataRef handles:
    drMatrixWrld   = XPLMFindDataRef("sim/graphics/view/world_matrix");
    drMatrixProj   = XPLMFindDataRef("sim/graphics/view/projection_matrix_3d");
    drScreenWidth  = XPLMFindDataRef("sim/graphics/view/window_width");
    drScreenHeight = XPLMFindDataRef("sim/graphics/view/window_height");
    drVisibility   = XPLMFindDataRef("sim/graphics/view/visibility_effective_m");
    if (!drVisibility)
        drVisibility    = XPLMFindDataRef("sim/weather/visibility_effective_m");
    drFieldOfView  = XPLMFindDataRef("sim/graphics/view/field_of_view_deg");
    
    // Register the drawing callback if need be
    if (glob.bDrawLabels)
        TwoDActivate();
}

// Grace cleanup
void TwoDCleanup ()
{
    // Remove drawing callbacks
    TwoDDeactivate();
}


}  // namespace XPMP2

//
// MARK: General API functions outside XPMP2 namespace
//

using namespace XPMP2;

// Enable/Disable/Query drawing of labels
void XPMPEnableAircraftLabels (bool _enable)
{
    // Label drawing overriden in global config?
    switch (glob.eLabelOverride) {
        case XPMP2::SWITCH_CFG_ON:
            LOG_MSG(logDEBUG, "Label drawing enforced ON in an XPMP2.prf config file");
            _enable = true;
            break;
        case XPMP2::SWITCH_CFG_OFF:
            LOG_MSG(logDEBUG, "Label drawing enforced OFF in an XPMP2.prf config file");
            _enable = false;
            break;
        case XPMP2::SWITCH_CFG_AUTO:
            break;
    }
    
    // Only do anything if this actually is a change to prevent log spamming
    if (glob.bDrawLabels != _enable) {
        LOG_MSG(logDEBUG, DEBUG_ENABLE_AC_LABELS, _enable ? "enabled" : "disabled");
        glob.bDrawLabels = _enable;
        
        // Start/stop drawing as requested
        if (glob.bDrawLabels)
            TwoDActivate();
        else
            TwoDDeactivate();
    }
}

void XPMPDisableAircraftLabels()
{
    XPMPEnableAircraftLabels(false);
}

bool XPMPDrawingAircraftLabels()
{
    return glob.bDrawLabels;
}

// Configure maximum label distance and if labels shall be cut off at reported visibility
void XPMPSetAircraftLabelDist (float _dist_nm, bool _bCutOffAtVisibility)
{
    glob.bLabelCutOffAtVisibility = _bCutOffAtVisibility;
    glob.maxLabelDist = std::max(_dist_nm,1.0f) * M_per_NM; // store in meter
}

