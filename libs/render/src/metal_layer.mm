#import <AppKit/AppKit.h>
#import <QuartzCore/CAMetalLayer.h>

#include "drape/render/renderer.hpp"

namespace drape::render {

void* makeMetalLayerForView(void* nsView) {
  NSView* view = (__bridge NSView*)nsView;
  CAMetalLayer* layer = [CAMetalLayer layer];
  layer.contentsScale = view.window ? view.window.backingScaleFactor : NSScreen.mainScreen.backingScaleFactor;
  layer.opaque = YES;
  view.layer = layer;  // layer-hosting view: set the layer, then wantsLayer
  view.wantsLayer = YES;
  return (__bridge void*)layer;
}

double backingScale(void* nsView) {
  NSView* view = (__bridge NSView*)nsView;
  return view.window ? view.window.backingScaleFactor : NSScreen.mainScreen.backingScaleFactor;
}

void setMetalLayerScale(void* caMetalLayer, double scale) {
  ((__bridge CAMetalLayer*)caMetalLayer).contentsScale = scale;
}

}  // namespace drape::render
