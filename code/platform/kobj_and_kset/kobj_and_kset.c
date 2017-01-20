/*		
 *	name:	kobj_and_kset.c
 *	auth:	wuw
 *	date:	2016年3月7日 
 *	
 *
 */



#include <linux/device.h>  
#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/init.h>  
#include <linux/string.h>  
#include <linux/sysfs.h>  
#include <linux/stat.h> 
#include <linux/slab.h>


struct kset *kset_a;
struct kset *kset_b;

struct my_kobj{
    int val;
    struct kobject kobj;
};

struct my_kobj *obj1;
struct my_kobj *obj2;
struct kobj_type my_type;


struct attribute name_attr = {	
	.name = "name",	
	.mode = 0444,
};


struct attribute val_attr = {	
	.name = "val",	
	.mode = 0666
};

struct attribute *my_attrs[] = {	
	&name_attr, 	
	&val_attr,	
	NULL		//一定要有NULL结尾
};

ssize_t my_show(struct kobject *kobj, struct attribute *attr, char *buffer)
{
	struct my_kobj *obj = container_of(kobj, struct my_kobj, kobj);
	ssize_t count = 0;
	if(strcmp(attr->name, "name") == 0)
		count = sprintf(buffer, "%s\n", kobject_name(kobj));
	else if(strcmp(attr->name, "val") == 0)
		count = sprintf(buffer, "%d\n", obj->val);

	return count;
}

ssize_t my_store(struct kobject *kobj, struct attribute *attr, const char *buffer, size_t size)
{
	struct my_kobj *obj = container_of(kobj, struct my_kobj, kobj);	
	if (strcmp(attr->name, "val") == 0) {		
		sscanf(buffer, "%d", &obj->val);	
	}	
	return size;
}


struct sysfs_ops my_sysfsops = {	
	.show = my_show,	
	.store = my_store

};

void obj_release(struct kobject *kobj)
{	
	struct my_kobj *obj = container_of(kobj, struct my_kobj, kobj);	
	printk(KERN_INFO "obj_release %s\n", kobject_name(&obj->kobj));	
	kfree(obj);
}



int kset_filter(struct kset *kset, struct kobject *kobj)
{
	printk(KERN_INFO "filter: kobj %s.\n", kobj->name);
	return 0;

}

const char *kset_name(struct kset *kset, struct kobject *kobj)
{
	static char buf[32];
	printk(KERN_INFO "name: kobj %s.\n", kobj->name);
	sprintf(buf, "%s", kset->kobj.name);
	return buf;

}
int kset_uevent(struct kset *kset, struct kobject *kobj, struct kobj_uevent_env *env)
{
	int i = 0;
	printk(KERN_INFO "uevent: kobj %s.\n", kobj->name);
	while(i < env->envp_idx){
		printk(KERN_INFO "env[%d]: %s.\n", i, env->envp[i]);
		i++;
	}
	return 0;

}

struct kset_uevent_ops uevent_ops = {
	.filter = kset_filter,
	.name = kset_name,
	.uevent = kset_uevent,

};
static int __init kobjandkset_test_init(void)
{
	int ret;
	printk(KERN_INFO "kboject register init.\n"); 
	
	//给两个kobject结构分配空间
	obj1 = kzalloc(sizeof(struct my_kobj), GFP_KERNEL);
	if(!obj1)
		return -ENOMEM;
	obj1->val = 1;

	obj2 = kzalloc(sizeof(struct my_kobj), GFP_KERNEL);
	if(!obj2)
		return -ENOMEM;
	obj2->val = 2;

	//为kobj_type设置属性、操作方法和释放方式
	my_type.release = obj_release;
	my_type.default_attrs = my_attrs;
	my_type.sysfs_ops = &my_sysfsops;

	//关联kobject和kobj_type,并添加到内核
	ret = kobject_init_and_add(&obj1->kobj, &my_type, NULL, "my_kobj1");
	ret = kobject_init_and_add(&obj2->kobj, &my_type, NULL, "my_kobj2");		


	printk(KERN_INFO "kset register init!\n");
	pr_info("+++++++++++++");
	kset_a = kset_create_and_add("kset_a", &uevent_ops, NULL);
	pr_info("+++++++++++++");
	kset_b = kset_create_and_add("kset_b", &uevent_ops, &kset_a->kobj);
	pr_info("+++++++++++++");




	return 0;

}

static void __exit kobjandkset_test_exit(void)
{
	printk(KERN_INFO "kboject test exit.\n");  

	kobject_del(&obj2->kobj);	
	kobject_put(&obj2->kobj);		
	kobject_del(&obj1->kobj);	
	kobject_put(&obj1->kobj);

	kset_unregister(kset_a);
	kset_unregister(kset_b);



}


module_init(kobjandkset_test_init);  
module_exit(kobjandkset_test_exit);  


MODULE_AUTHOR("yshisx");  
MODULE_LICENSE("Dual BSD/GPL");






