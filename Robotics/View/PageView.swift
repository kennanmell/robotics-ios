//
//  PageView.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class PageView: UIView {
    var buttonArray = Array<UIButton>()
    let textLabel = UILabel()
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        
        self.textLabel.textAlignment = .center
        self.textLabel.font = UIFont(name: self.textLabel.font.fontName, size: 20)
        self.textLabel.numberOfLines = 0
        self.textLabel.adjustsFontSizeToFitWidth = true
        self.addSubview(textLabel)
    }
    
    override func layoutSubviews() {
        var yOffset = self.frame.height * 0.95 - self.frame.width * 0.15
        
        for i in (0..<buttonArray.count).reversed() {
            self.buttonArray[i].frame = CGRect(x: self.frame.width * 0.1,
                                               y: yOffset,
                                               width: self.frame.width * 0.8,
                                               height: self.frame.width * 0.15)
            yOffset -= self.frame.width * 0.2
        }
        
        self.textLabel.frame = CGRect(x: self.frame.width * 0.1,
                                      y: self.frame.width * 0.3,
                                      width: self.frame.width * 0.8,
                                      height: yOffset - self.frame.width * 0.3)
    }
    
    func addButton(button: UIButton) {
        if self.buttonArray.count >= 5 {
            fatalError("Tried to add too many buttons")
        }
        button.backgroundColor =
            UIColor(red: 0, green: 0, blue: 200.0 / 255.0, alpha: 1.0)
        button.showsTouchWhenHighlighted = true
        button.layer.cornerRadius = 5.0
        button.layer.borderWidth = 3.0
        button.layer.borderColor = UIColor.clear.cgColor
        button.layer.shadowColor =
            UIColor(red: 0, green: 0, blue: 100.0 / 255.0, alpha: 1.0).cgColor
        button.layer.shadowOpacity = 1.0
        button.layer.shadowRadius = 1.0
        button.layer.shadowOffset = CGSize(width: 0, height: 3)
        //button.setTitle("Cancel", for: .normal)
        
        self.addSubview(button)
        self.buttonArray.append(button)
        self.setNeedsLayout()
    }
}
