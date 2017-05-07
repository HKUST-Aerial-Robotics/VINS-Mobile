
#import "PJRSignatureView.h"


#define INITIAL_COLOR [UIColor redColor]; // Initial color for line  drawing.
#define FINAL_COLOR [UIColor redColor];// End color after completd drawing

#define INITIAL_LABEL_TEXT @"";


@implementation PJRSignatureView
{
    UIBezierPath *beizerPath;
    UIImage *incrImage;
    CGPoint points[5];
    uint control;
    CGContextRef _context;
    
    CGPoint cur_point;
    CGPoint pre_point;
    bool init_ok;
}

// Create a View which contains Signature Label

- (id)initWithFrame:(CGRect)frame
{
    self = [super initWithFrame:frame];
    
    if (self) {
        
        float lblHeight = 61;
        self.backgroundColor = [UIColor whiteColor];
        [self setMultipleTouchEnabled:NO];
        beizerPath = [UIBezierPath bezierPath];
        [beizerPath setLineWidth:2.0];
        lblSignature = [[UILabel alloc] initWithFrame:CGRectMake(0, self.frame.size.height/2 - lblHeight/2, self.frame.size.width, lblHeight)];
        lblSignature.font = [UIFont fontWithName:@"HelveticaNeue" size:51];
        lblSignature.text = INITIAL_LABEL_TEXT;
        lblSignature.textColor = [UIColor lightGrayColor];
        lblSignature.textAlignment = NSTextAlignmentCenter;
        lblSignature.alpha = 0.3;
        [self addSubview:lblSignature];
        init_ok = false;
    }
    return self;
}


// Only override drawRect: if you perform custom drawing.
// An empty implementation adversely affects performance during animation.
- (void)drawRect:(CGRect)rect
{
    [incrImage drawInRect:rect];
    [beizerPath stroke];
    
    // Set initial color for drawing
    
    UIColor *fillColor = INITIAL_COLOR;
    [fillColor setFill];
    UIColor *strokeColor = INITIAL_COLOR;
    [strokeColor setStroke];
    [beizerPath stroke];
}

#pragma mark - UIView Touch Methods

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
    if ([lblSignature superview]){
        [lblSignature removeFromSuperview];
    }
    control = 0;
    UITouch *touch = [touches anyObject];
    points[0] = [touch locationInView:self];
    
    CGPoint startPoint = points[0];
    CGPoint endPoint = CGPointMake(startPoint.x + 1.5, startPoint.y
                              + 2);
    
    [beizerPath moveToPoint:startPoint];
    [beizerPath addLineToPoint:endPoint];
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    UITouch *touch = [touches anyObject];
    CGPoint touchPoint = [touch locationInView:self];
    control++;
    points[control] = touchPoint;
    
    /*
    if (control == 4)
    {
        points[3] = CGPointMake((points[2].x + points[4].x)/2.0, (points[2].y + points[4].y)/2.0);
        
        [beizerPath moveToPoint:points[0]];
        [beizerPath addCurveToPoint:points[3] controlPoint1:points[1] controlPoint2:points[2]];
        
        [self setNeedsDisplay];
        
        points[0] = points[3];
        points[1] = points[4];
        control = 1;
    }
    */
    [beizerPath moveToPoint:points[0]];
    [beizerPath addLineToPoint:points[1]];
    [self setNeedsDisplay];
    control = 0;
    points[0] = points[1];
    
}

- (void)update_point:(double)x y:(double)y
{
    cur_point = CGPointMake(50*x+70,50*y+140);
    if(!init_ok)
    {
        pre_point = cur_point;
        init_ok = true;
    }
    printf("view: %.2lf, %.2lf\n",cur_point.x,cur_point.y);
    if ([lblSignature superview]){
        [lblSignature removeFromSuperview];
    }

    [beizerPath moveToPoint:pre_point];
    [beizerPath addLineToPoint:cur_point];
    [self setNeedsDisplay];
    pre_point = cur_point;
    UIColor *fillColor = INITIAL_COLOR;
    [fillColor setFill];
    UIColor *strokeColor = INITIAL_COLOR;
    [strokeColor setStroke];

    [beizerPath stroke];
    [beizerPath fill];
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
    [self drawBitmapImage];
    [self setNeedsDisplay];
    [beizerPath removeAllPoints];
    control = 0;
}

- (void)touchesCancelled:(NSSet *)touches withEvent:(UIEvent *)event
{
    [self touchesEnded:touches withEvent:event];
}

#pragma mark - Bitmap Image Creation

- (void)drawBitmapImage
{
    UIGraphicsBeginImageContextWithOptions(self.bounds.size, YES, 0.0);
    
    if (!incrImage)
    {
        UIBezierPath *rectpath = [UIBezierPath bezierPathWithRect:self.bounds];
        [[UIColor whiteColor] setFill];
        [rectpath fill];
    }
    [incrImage drawAtPoint:CGPointZero];
    
    //Set final color for drawing
    UIColor *strokeColor = FINAL_COLOR;
    [strokeColor setStroke];
    [beizerPath stroke];
    incrImage = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
}

- (void)clearSignature
{
    incrImage = nil;
    [self setNeedsDisplay];
}

#pragma mark - Get Signature image from given path

- (UIImage *)getSignatureImage {
    
    if([lblSignature superview]){
        return nil;
    }
    UIGraphicsBeginImageContextWithOptions(self.bounds.size, NO, [UIScreen mainScreen].scale);
    
    [self drawViewHierarchyInRect:self.bounds afterScreenUpdates:YES];
    
    UIImage *signatureImage = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
    return signatureImage;
}


@end

