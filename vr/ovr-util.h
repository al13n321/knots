#pragma once

#include "OVR_CAPI_GL.h"
#include "Extras/OVR_Math.h"
#include <array>
#include <cassert>
#include <iostream>
#include <sstream>
#include "util/quat.h"
#include "gl-util/texture2d.h"

namespace ovr {

  void PrintError() {
    ovrErrorInfo info;
    ovr_GetLastErrorInfo(&info);
    std::cerr << "OVR error: " << info.ErrorString << std::endl;
  }

  bool PrintIfError(ovrResult r) {
    if (!OVR_SUCCESS(r)) {
      PrintError();
      return true;
    }
    return false;
  }

  struct Pose {
    dvec3 position;
    dquat orientation;
    dvec3 velocity;
    dvec3 angular_velocity;
  };

  struct HandState {
    Pose pose;
    int status_flags; // ovrStatusBits
    ovrInputState input;
  };

  struct EyeState {
    ovrPosef pose;
    fmat4 view_projection;
  };

  struct FrameInfo {
    int64_t frame_idx;
    double begin_time;
    double predicted_display_time;
    ovrTimewarpProjectionDesc timewarp_projection_desc;

    ovrPoseStatef head_pose;
    std::array<EyeState, 2> eyes;
    std::array<HandState, 2> hands;
  };

  struct Session {
    struct PerEyeStuff {
      ovrFovPort fov;
      ovrSizei size;
      ovrTextureSwapChain color_swap_chain = nullptr;
      ovrTextureSwapChain depth_swap_chain = nullptr;
      GLuint fbo = 0;
    };

    ovrSession session = nullptr;
    ovrGraphicsLuid graphics_adapter_luid;
    std::array<PerEyeStuff, 2> eyes;

    bool inited_ovr = false;

    bool Init() {
      ovrInitParams params = {};
      params.Flags = ovrInit_Debug;
      auto r = ovr_Initialize(&params);
      if (PrintIfError(r)) return false;
      inited_ovr = true;

      r = ovr_Create(&session, &graphics_adapter_luid);
      if (PrintIfError(r)) return false;

      r = ovr_SetTrackingOriginType(session, ovrTrackingOrigin_FloorLevel);
      if (PrintIfError(r)) return false;

      ovrHmdDesc hmd_desc = ovr_GetHmdDesc(session);
      std::cerr << "HMD name: " << hmd_desc.ProductName << '\n';
      std::cerr << "HMD manufacturer: " << hmd_desc.Manufacturer << '\n';
      std::cerr << "HMD vendor id: " << hmd_desc.VendorId << ", Product Id: " << hmd_desc.ProductId << '\n';
      std::cerr << "HMD serial number: " << hmd_desc.SerialNumber << '\n';
      std::cerr << "HMD firmware: " << hmd_desc.FirmwareMajor << '.' << hmd_desc.FirmwareMinor << '\n';
      std::cerr << "HMD various bitmasks (see source code): " << hmd_desc.AvailableHmdCaps << ", " << hmd_desc.DefaultHmdCaps << ", " << hmd_desc.AvailableTrackingCaps << ", " << hmd_desc.DefaultTrackingCaps << '\n';
      auto fov_str = [](ovrFovPort p) {
        std::stringstream ss;
        ss << p.UpTan << "+" << p.DownTan << ", " << p.LeftTan << "+" << p.RightTan;
        return ss.str();
      };
      std::cerr << "HMD default FOV: " << fov_str(hmd_desc.DefaultEyeFov[0]) << ", " << fov_str(hmd_desc.DefaultEyeFov[1]) << '\n';
      std::cerr << "HMD max FOV: " << fov_str(hmd_desc.MaxEyeFov[0]) << ", " << fov_str(hmd_desc.MaxEyeFov[1]) << '\n';
      std::cerr << "HMD resolution: " << hmd_desc.Resolution.w << ", " << hmd_desc.Resolution.h << '\n';
      std::cerr << "HMD refresh rate: " << hmd_desc.DisplayRefreshRate << '\n';
      if (hmd_desc.Type == ovrHmd_None) {
        PrintError();
        return false;
      }
      for (size_t ei = 0; ei < 2; ++ei) {
        PerEyeStuff& eye = eyes[ei];
        eye.fov = hmd_desc.DefaultEyeFov[ei];
        eye.size = ovr_GetFovTextureSize(session, ovrEyeType(ei), eye.fov, 1.f);
      }

      return true;
    }

    // Call after initializing OpenGL.
    bool InitSwapChains() {
      for (size_t ei = 0; ei < 2; ++ei) {
        PerEyeStuff& eye = eyes[ei];

        ovrTextureSwapChainDesc desc;
        memset(&desc, 0, sizeof(desc));
        desc.Type = ovrTexture_2D;
        desc.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
        desc.ArraySize = 1;
        desc.Width = eye.size.w;
        desc.Height = eye.size.h;
        desc.MipLevels = 1;
        desc.SampleCount = 1;

        auto r = ovr_CreateTextureSwapChainGL(
          session,
          &desc,
          &eye.color_swap_chain);
        if (PrintIfError(r)) return false;

        desc.Format = OVR_FORMAT_D32_FLOAT;

        r = ovr_CreateTextureSwapChainGL(
          session,
          &desc,
          &eye.depth_swap_chain);
        if (PrintIfError(r)) return false;

        glGenFramebuffers(1, &eye.fbo);
        if (SOFT_CHECK_GL_ERROR()) {
          assert(!eye.fbo);
          return false;
        }
      }

      return true;
    }

    // Has to be called before destroying OpenGL.
    void DestroySwapChains() {
      for (size_t ei = 0; ei < 2; ++ei) {
        PerEyeStuff& eye = eyes[ei];
        if (eye.fbo) {
          glDeleteFramebuffers(1, &eye.fbo);
          eye.fbo = 0;
        }
        if (eye.depth_swap_chain) ovr_DestroyTextureSwapChain(session, std::exchange(eye.depth_swap_chain, nullptr));
        if (eye.color_swap_chain) ovr_DestroyTextureSwapChain(session, std::exchange(eye.color_swap_chain, nullptr));
      }
    }

    ~Session() {
      DestroySwapChains();
      if (session) ovr_Destroy(session);
      if (inited_ovr) ovr_Shutdown();
    }

    static dvec3 ConvertVec3(ovrVector3f v) {
      return dvec3(v.x, v.y, v.z);
    }
    static dquat ConvertQuat(ovrQuatf q) {
      return dquat(q.w, q.x, q.y, q.z);
    }
    static Pose ConvertPose(ovrPoseStatef p) {
      Pose r;
      r.position = ConvertVec3(p.ThePose.Position);
      r.orientation = ConvertQuat(p.ThePose.Orientation);
      r.velocity = ConvertVec3(p.LinearVelocity);
      r.angular_velocity = ConvertVec3(p.AngularVelocity);
      return r;
    }
    static fmat4 ConvertMat4(OVR::Matrix4f m) {
      fmat4 r;
      for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) r.m[i * 4 + j] = m.M[i][j];
      return r;
    }

    bool BeginFrame(int64_t frame_idx, FrameInfo& out_frame) {
      out_frame.frame_idx = frame_idx;

      ovrSessionStatus sessionStatus;
      ovr_GetSessionStatus(session, &sessionStatus);
      if (sessionStatus.ShouldQuit) return false;

      auto r = ovr_WaitToBeginFrame(session, frame_idx);
      if (PrintIfError(r)) return false;
      r = ovr_BeginFrame(session, frame_idx);
      if (PrintIfError(r)) return false;

      out_frame.begin_time = ovr_GetTimeInSeconds();
      out_frame.predicted_display_time = ovr_GetPredictedDisplayTime(session, out_frame.frame_idx);

      auto s = ovr_GetTrackingState(session, out_frame.predicted_display_time, true);
      out_frame.head_pose = s.HeadPose;
      for (size_t i = 0; i < 2; ++i) {
        HandState& h = out_frame.hands[i];
        h.pose = ConvertPose(s.HandPoses[i]);
        h.status_flags = s.HandStatusFlags[i];
        auto r = ovr_GetInputState(session, i ? ovrControllerType_RTouch : ovrControllerType_LTouch, &h.input);
        if (!OVR_SUCCESS(r)) {
          if (r != ovrError_LostTracking) PrintError();
        }
      }

      {
        ovrEyeRenderDesc descs[2];
        descs[0] = ovr_GetRenderDesc(session, ovrEye_Left, eyes[0].fov);
        descs[1] = ovr_GetRenderDesc(session, ovrEye_Right, eyes[1].fov);

        ovrPosef temp_poses[2] = { descs[0].HmdToEyePose, descs[1].HmdToEyePose };
        ovrPosef poses[2];
        ovr_CalcEyePoses2(out_frame.head_pose.ThePose, temp_poses, poses);

        for (size_t ei = 0; ei < 2; ++ei) {
          EyeState& eye = out_frame.eyes[ei];
          eye.pose = poses[ei];
          OVR::Matrix4f proj = ovrMatrix4f_Projection(descs[ei].Fov, .1f, 1e3f, 0);
          eye.view_projection = ConvertMat4(proj * OVR::Matrix4f(OVR::Posef(eye.pose).Inverted()));

          if (ei == 0) {
            // It's weird that non-per-eye data is produced from per-eye data of an arbitrary eye,
            // but AFAICT this is the intended way to do this.
            out_frame.timewarp_projection_desc = ovrTimewarpProjectionDesc_FromProjection(proj, 0);
          }
        }
      }

      return true;
    }

    bool SetEyeRenderTarget(size_t ei) {
      PerEyeStuff& eye = eyes[ei];

      GLuint color_texture;
      GLuint depth_texture;
      {
        int idx;
        auto r = ovr_GetTextureSwapChainCurrentIndex(session, eye.color_swap_chain, &idx);
        if (PrintIfError(r)) return false;
        r = ovr_GetTextureSwapChainBufferGL(session, eye.color_swap_chain, idx, &color_texture);
        if (PrintIfError(r)) return false;
      }
      {
        int idx;
        auto r = ovr_GetTextureSwapChainCurrentIndex(session, eye.depth_swap_chain, &idx);
        if (PrintIfError(r)) return false;
        r = ovr_GetTextureSwapChainBufferGL(session, eye.depth_swap_chain, idx, &depth_texture);
        if (PrintIfError(r)) return false;
      }

      glBindFramebuffer(GL_FRAMEBUFFER, eye.fbo);
      if (SOFT_CHECK_GL_ERROR()) return false;
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_texture, 0);
      if (SOFT_CHECK_GL_ERROR()) return false;
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_texture, 0);
      if (SOFT_CHECK_GL_ERROR()) return false;

      glViewport(0, 0, eye.size.w, eye.size.h);
      if (SOFT_CHECK_GL_ERROR()) return false;
      glEnable(GL_FRAMEBUFFER_SRGB);
      if (SOFT_CHECK_GL_ERROR()) return false;

      return true;
    }

    bool CommitEye(size_t ei) {
      PerEyeStuff& eye = eyes[ei];

      glBindFramebuffer(GL_FRAMEBUFFER, eye.fbo);
      if (SOFT_CHECK_GL_ERROR()) return false;
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
      if (SOFT_CHECK_GL_ERROR()) return false;
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
      if (SOFT_CHECK_GL_ERROR()) return false;

      auto r= ovr_CommitTextureSwapChain(session, eye.color_swap_chain);
      if (PrintIfError(r)) return false;
      r = ovr_CommitTextureSwapChain(session, eye.depth_swap_chain);
      if (PrintIfError(r)) return false;

      return true;
    }

    bool EndFrame(FrameInfo& frame) {
      ovrLayerEyeFovDepth layer = {};
      layer.Header.Type = ovrLayerType_EyeFovDepth;
      layer.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft;
      layer.SensorSampleTime = frame.begin_time;
      layer.ProjectionDesc = frame.timewarp_projection_desc;
      for (size_t ei = 0; ei < 2; ++ei) {
        PerEyeStuff& eye = eyes[ei];
        layer.ColorTexture[ei] = eye.color_swap_chain;
        layer.DepthTexture[ei] = eye.depth_swap_chain;
        layer.Fov[ei] = eye.fov;

        layer.Viewport[ei] = OVR::Recti(eye.size);
        layer.RenderPose[ei] = frame.eyes[ei].pose;
      }
      auto* layer_ptr = &layer.Header;

      auto r = ovr_EndFrame(session, frame.frame_idx, nullptr, &layer_ptr, 1);
      if (!OVR_SUCCESS(r)) {
        PrintError();
        return false;
      }

      return true;
    }
  };
}
