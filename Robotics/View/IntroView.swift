//
//  IntroView.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class IntroView: PageView {
    let noServerView = NoServerView()
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        self.noServerView.isHidden = true
        self.addSubview(noServerView)
    }
    
    override func layoutSubviews() {
        super.layoutSubviews()
        noServerView.frame = self.frame
    }
}
