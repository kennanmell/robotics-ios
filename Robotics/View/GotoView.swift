//
//  GotoView.swift
//  Robotics
//
//  Created by Kennan Mell on 3/3/18.
//  Copyright © 2018 Kennan Mell. All rights reserved.
//

import UIKit

class GotoView: PageView {
    let tableView = UITableView()
    
    required init?(coder aDecoder: NSCoder) {
        super.init(coder: aDecoder)
        self.tableView.layer.cornerRadius = 5.0
        self.tableView.layer.borderWidth = 3.0
        self.tableView.layer.borderColor = UIColor.darkGray.cgColor
        self.addSubview(self.tableView)
    }
    
    override func layoutSubviews() {
        super.layoutSubviews()
        
        let tableHeight = self.frame.height * 0.95 - self.frame.width * 0.15 -
            self.frame.width * 0.2 * CGFloat(self.buttonArray.count) -
            self.frame.width * 0.6
        
        self.textLabel.frame = CGRect(x: self.frame.width * 0.1,
                                      y: self.frame.width * 0.3,
                                      width: self.frame.width * 0.8,
                                      height: self.frame.width * 0.3)
        
        self.tableView.frame = CGRect(x: self.frame.width * 0.1,
                                      y: self.frame.width * 0.6,
                                      width: self.frame.width * 0.8,
                                      height: tableHeight)
    }
}

