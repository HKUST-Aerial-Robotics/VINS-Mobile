
#import <UIKit/UIKit.h>

@interface PJRSignatureView : UIView
{
    UILabel *lblSignature;
    CAShapeLayer *shapeLayer;
}

- (UIImage *)getSignatureImage;
- (void)clearSignature;
- (void)update_point:(double)x y:(double)y;


@end

