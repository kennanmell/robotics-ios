//
//  FindMeView.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class FindMeView: PageView {
    let tagImageView = UIImageView(image: UIImage(named: "ar_tag"))
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        self.addSubview(self.tagImageView)
        self.sendSubview(toBack: self.tagImageView)
    }
    
    override func layoutSubviews() {
        super.layoutSubviews()
        
        var yOffset = self.frame.height * 0.95 - self.frame.width * 0.15 -
            self.frame.width * 0.2 * CGFloat(self.buttonArray.count - 1) -
            self.frame.width * 0.9
        
        if UIScreen.main.nativeBounds.height == 2436 {
            // iPhone X
            yOffset -= self.frame.width * 0.1
        }
        
        self.tagImageView.frame = CGRect(x: 0,
                                         y: yOffset,
                                         width: self.frame.width,
                                         height: self.frame.width)
        
        self.textLabel.frame = CGRect(x: self.frame.width * 0.1,
                                      y: self.frame.width * 0.3,
                                      width: self.frame.width * 0.8,
                                      height: yOffset - self.frame.width * 0.3)
    }
}
