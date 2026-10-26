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

void resizeMetalLayer(void* caMetalLayer, double scale, std::uint32_t widthPx, std::uint32_t heightPx) {
  CAMetalLayer* layer = (__bridge CAMetalLayer*)caMetalLayer;
  layer.contentsScale = scale;
  layer.drawableSize = CGSizeMake(widthPx, heightPx);
}

}  // namespace drape::render
